#!/usr/bin/env python3
"""OSC-driven CrazySwarm2 bridge that understands a full swarm, not just one whoop."""

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from functools import partial
from typing import Any, Dict, Iterable, List, Optional, Tuple

from pythonosc import dispatcher, osc_server, udp_client

try:
    from software.swarm.mapping_loader import load_mapping
    from software.swarm.virtual_swarm import (
        VirtualDrone,
        enforce_min_separation,
        minimum_pairwise_distance,
    )
except ModuleNotFoundError:  # pragma: no cover - script-path fallback
    from mapping_loader import load_mapping
    from virtual_swarm import (
        VirtualDrone,
        enforce_min_separation,
        minimum_pairwise_distance,
    )

ROS_IMPORT_ERROR: Optional[Exception] = None
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message
    from crazyflie_interfaces.srv import GoTo, Land, Stop, Takeoff

    NodeBase = Node
except Exception as exc:  # pragma: no cover - exercised only on non-ROS hosts
    ROS_IMPORT_ERROR = exc
    rclpy = None
    NodeBase = object
    QoSHistoryPolicy = QoSProfile = QoSReliabilityPolicy = None
    message_to_ordereddict = None
    get_message = None
    GoTo = Land = Stop = Takeoff = None

# ---------------------------------------------------------------------------
# Fleet layout + OSC wiring — tweak these to match your venue rig.
# ---------------------------------------------------------------------------

OSC_BIND_ADDRESS = "127.0.0.1"
OSC_BIND_PORT = 9010
DEFAULT_CONSENT_TIMEOUT_S = 1.0

DEFAULT_BASE_MAPPING_PATH = "config/mapping.yaml"
DEFAULT_RECIPES_DIR = "config/recipes"
SIM_OSC_TARGET = "127.0.0.1"
SIM_OSC_PORT = 9100
CONSENT_STATE_BROADCAST = "/pd/current_consent"
SIM_BENCH_PING_ROUTE = "/pd/bench/ping"
SIM_BENCH_ACK_ROUTE = "/pd/sim/bench/ack"
SIM_COLLISION_ROUTE = "/pd/sim/collision/min_distance"
SIM_MIN_SEPARATION_M = 0.35

# Global OSC routes still exist so legacy Processing sketches keep working.
GLOBAL_OSC_ROUTES = {
    "lat": "/pd/lat",
    "alt": "/pd/alt",
}


# Per-drone OSC routes let you break formation and address craft individually.
@dataclass(frozen=True)
class CraftConfig:
    name: str
    go_to_service: str
    takeoff_service: str
    osc_addresses: Dict[str, str]
    land_service: Optional[str] = None
    stop_service: Optional[str] = None
    telemetry_topic: Optional[str] = None
    telemetry_type: Optional[str] = None


DEFAULT_FLEET: List[CraftConfig] = [
    CraftConfig(
        name="cf1",
        go_to_service="/cf1/go_to",
        takeoff_service="/cf1/takeoff",
        land_service="/cf1/land",
        stop_service="/cf1/stop",
        osc_addresses={
            "lat": "/pd/cf1/lat",
            "alt": "/pd/cf1/alt",
        },
        telemetry_topic="/cf1/telemetry",
        telemetry_type="crazyflie_interfaces/msg/LogDataGeneric",
    ),
    CraftConfig(
        name="cf2",
        go_to_service="/cf2/go_to",
        takeoff_service="/cf2/takeoff",
        land_service="/cf2/land",
        stop_service="/cf2/stop",
        osc_addresses={
            "lat": "/pd/cf2/lat",
            "alt": "/pd/cf2/alt",
        },
        telemetry_topic="/cf2/telemetry",
        telemetry_type="crazyflie_interfaces/msg/LogDataGeneric",
    ),
]

# Lateral motion scaling: incoming [-1, 1] values become offsets in meters.
LATERAL_INPUT_RANGE = (-1.0, 1.0)
LATERAL_HOME_M = 0.0
LATERAL_SCALE_M = 0.7
LATERAL_AXIS = "y"  # choose "x" or "y" depending on your room orientation

# Altitude: convert normalized inputs into safe takeoff heights (meters).
ALTITUDE_INPUT_RANGE = (0.0, 1.0)
ALTITUDE_FLOOR_M = 0.3
ALTITUDE_SCALE_M = 0.9


def _binary_consent_state(value: Any, default: int = 0) -> int:
    """Normalize consent-ish values into the shared 0/1 rule."""

    if isinstance(value, bool):
        return int(value)
    if isinstance(value, (int, float)):
        return 1 if float(value) >= 0.5 else 0
    return int(default)


def _is_loopback_bind(host: str) -> bool:
    return host.strip().lower() in {"127.0.0.1", "::1", "localhost"}


def _warn_if_exposed_bind(host: str, port: int) -> Optional[str]:
    if _is_loopback_bind(host):
        return None
    warning = (
        "WARNING: swarm OSC control is exposed on "
        f"{host}:{port}. Any reachable host can submit consent, recipe, and "
        "motion commands. Use only on a physically isolated, trusted control "
        "network."
    )
    print(warning, file=sys.stderr)
    return warning


# Motion timing and default metadata.  Set RELATIVE_MOVES to True if your GoTo
# service expects relative offsets instead of absolute coordinates.
GO_TO_DURATION_S = 0.5
TAKEOFF_DURATION_S = 1.5
LAND_DURATION_S = 2.0
GROUP_MASK = 0
RELATIVE_MOVES = False

# Telemetry snapshots print every few seconds so operators can see the roundtrip.
STATUS_REPORT_INTERVAL_S = 5.0


@dataclass
class CraftState:
    config: CraftConfig
    go_to_client: Any = field(repr=False)
    takeoff_client: Any = field(repr=False)
    land_client: Any = field(default=None, repr=False)
    stop_client: Any = field(default=None, repr=False)
    current_altitude: float = ALTITUDE_FLOOR_M
    current_lateral: float = LATERAL_HOME_M
    formation_offset: float = 0.0
    telemetry: Optional[Dict[str, object]] = None


class SwarmNode(NodeBase):
    """ROS 2 node that listens for OSC and relays to a CrazySwarm2 fleet."""

    def __init__(
        self,
        fleet: Optional[List[CraftConfig]] = None,
        mapping: Optional[Dict[str, Any]] = None,
        recipe_name: Optional[str] = None,
        base_mapping_path: str = DEFAULT_BASE_MAPPING_PATH,
        recipes_dir: str = DEFAULT_RECIPES_DIR,
        osc_bind: str = OSC_BIND_ADDRESS,
        osc_port: int = OSC_BIND_PORT,
        consent_timeout_s: float = DEFAULT_CONSENT_TIMEOUT_S,
    ):
        if any(item is None for item in (rclpy, GoTo, Land, Stop, Takeoff)):
            raise RuntimeError(
                "Real swarm mode requires ROS2 + CrazySwarm2 dependencies. "
                f"Import failure: {ROS_IMPORT_ERROR}"
            )
        super().__init__("swarm_node")
        self.fleet_cfg = fleet or DEFAULT_FLEET
        if not self.fleet_cfg:
            raise RuntimeError(
                "At least one CraftConfig is required to run the swarm bridge"
            )

        self.base_mapping_path = base_mapping_path
        self.recipes_dir = recipes_dir
        self.mapping = mapping or load_mapping(
            base_path=self.base_mapping_path, recipes_dir=self.recipes_dir
        )
        self.current_recipe_name = recipe_name
        self._mapping_lock = threading.Lock()
        self._consent_lock = threading.Lock()
        self._consent_state = self._get_default_consent_state(self.mapping)
        self._last_consent_at: Optional[float] = None
        self._consent_timeout_s = max(0.1, float(consent_timeout_s))
        self._last_broadcast_consent: Optional[int] = None

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10,
        )

        self.dispatcher = dispatcher.Dispatcher()
        self.dispatcher.map(GLOBAL_OSC_ROUTES["lat"], self._handle_global_lat)
        self.dispatcher.map(GLOBAL_OSC_ROUTES["alt"], self._handle_global_alt)
        self.dispatcher.map("/pd/patch", self._handle_patch)
        self.dispatcher.map(self._consent_address(), self._handle_consent)
        self.dispatcher.map("/pd/consent", self._handle_consent)

        self.crafts: Dict[str, CraftState] = {}
        self._last_status_snapshot: Optional[str] = None
        self.gesture_state: Dict[str, float] = {
            "consent": float(self._consent_state)
        }

        span = max(len(self.fleet_cfg) - 1, 0)
        for index, cfg in enumerate(self.fleet_cfg):
            go_to_client = self.create_client(GoTo, cfg.go_to_service)
            takeoff_client = self.create_client(Takeoff, cfg.takeoff_service)
            land_service = cfg.land_service or f"/{cfg.name}/land"
            stop_service = cfg.stop_service or f"/{cfg.name}/stop"
            land_client = self.create_client(Land, land_service)
            stop_client = self.create_client(Stop, stop_service)
            formation_offset = (index - span / 2.0) * 0.6
            self.crafts[cfg.name] = CraftState(
                config=cfg,
                go_to_client=go_to_client,
                takeoff_client=takeoff_client,
                land_client=land_client,
                stop_client=stop_client,
                formation_offset=formation_offset,
            )
            for axis, address in cfg.osc_addresses.items():
                if axis not in {"lat", "alt"}:
                    continue
                handler = partial(self._handle_direct_command, cfg.name, axis)
                self.dispatcher.map(address, handler)
            if cfg.telemetry_topic and cfg.telemetry_type:
                self._wire_telemetry(cfg, qos)

        bind_warning = _warn_if_exposed_bind(osc_bind, osc_port)
        if bind_warning:
            self.get_logger().warning(bind_warning)
        self.server = osc_server.ThreadingOSCUDPServer(
            (osc_bind, osc_port),
            self.dispatcher,
        )
        self._osc_thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self._osc_thread.start()
        self.get_logger().info(
            "OSC listening on %s:%d for %s",
            *self.server.server_address,
            ", ".join(self.crafts.keys()),
        )
        self.get_logger().info(
            "Consent heartbeat timeout: %.2fs", self._consent_timeout_s
        )
        swarm_cfg = (
            self.mapping.get("swarm", {})
            if isinstance(self.mapping, dict)
            else {}
        )
        self._min_separation_m = float(
            swarm_cfg.get("min_separation_m", SIM_MIN_SEPARATION_M)
        )
        self._last_collision_warn_at = 0.0

        self.create_timer(STATUS_REPORT_INTERVAL_S, self._report_status)
        watchdog_interval = min(0.25, self._consent_timeout_s / 2.0)
        self.create_timer(watchdog_interval, self._expire_stale_consent)

    # ------------------------------------------------------------------
    # Consent helpers — treat participation as a first-class mapped signal.
    # ------------------------------------------------------------------
    def _consent_address(self) -> str:
        """Return the OSC address for consent, defaulting to /pd/consent."""

        if isinstance(self.mapping, dict):
            osc_cfg = self.mapping.get("osc", {}) or {}
            address_space = osc_cfg.get("address_space", {}) or {}
            return address_space.get("consent", "/pd/consent")
        return "/pd/consent"

    def _get_consent_config(self) -> Dict[str, Any]:
        with self._mapping_lock:
            mapping = self.mapping
        if not isinstance(mapping, dict):
            return {}
        consent_cfg = mapping.get("consent", {}) or {}
        return consent_cfg if isinstance(consent_cfg, dict) else {}

    def _get_default_consent_state(self, mapping: Dict[str, Any]) -> int:
        cfg = mapping.get("consent", {}) if isinstance(mapping, dict) else {}
        return _binary_consent_state(cfg.get("default_state", 0))

    def _expire_stale_consent(self, now: Optional[float] = None) -> bool:
        """Atomically turn stale live consent into the normal OFF transition."""

        checked_at = time.monotonic() if now is None else now
        with self._consent_lock:
            if self._consent_state != 1:
                return False
            if (
                self._last_consent_at is not None
                and checked_at - self._last_consent_at
                <= self._consent_timeout_s
            ):
                return False
            previous = self._consent_state
            self._consent_state = 0
            self._last_consent_at = None
            self.gesture_state["consent"] = 0.0

        self.get_logger().error(
            "[consent] tracker heartbeat stale for more than %.2fs; "
            "forcing OFF and landing/stopping all craft",
            self._consent_timeout_s,
        )
        self._handle_consent_edge(previous, 0)
        return True

    def _has_fresh_consent(self) -> bool:
        self._expire_stale_consent()
        with self._consent_lock:
            return (
                self._consent_state == 1 and self._last_consent_at is not None
            )

    def _handle_consent_edge(self, previous: int, current: int) -> None:
        """React to consent transitions: logging, auto-recipes, idling."""

        if previous == 1 and current == 0:
            self._land_and_stop_all("consent-off")

        consent_cfg = self._get_consent_config()
        auto_recipes = consent_cfg.get("auto_recipes", False)
        if auto_recipes:
            recipe_key = "on_recipe" if current else "off_recipe"
            recipe_target = (consent_cfg.get(recipe_key) or "").strip()
            if recipe_target and recipe_target.lower() != "none":
                try:
                    new_mapping = load_mapping(
                        base_path=self.base_mapping_path,
                        recipe_name=recipe_target,
                        recipes_dir=self.recipes_dir,
                    )
                except (
                    Exception
                ) as exc:  # pragma: no cover - defensive edge logging
                    self.get_logger().warning(
                        "Auto-recipe: failed to load '%s' on consent %s (%s)",
                        recipe_target,
                        "ON" if current else "OFF",
                        exc,
                    )
                else:
                    with self._mapping_lock:
                        self.mapping = new_mapping
                        self.current_recipe_name = recipe_target
                    self.get_logger().info(
                        "Auto-recipe: switched to '%s' on consent %s",
                        recipe_target,
                        "ON" if current else "OFF",
                    )
        # Manual /pd/patch calls can override whatever auto-recipes pick; the
        # consent edges only fire on transitions so they will not spam patches.

        # Always share the state with downstream consumers for clarity.
        self._broadcast_consent_state(current)

    def _apply_behavior_from_gestures(self, source: str) -> None:
        """Map incoming gestures to behavior and push to the fleet.

        This keeps the consent gate consistent between the simulated swarm and
        the live Crazyflie bridge without hard-coding idle posture numbers.
        """

        with self._mapping_lock:
            mapping = self.mapping

        behavior = _map_gestures_to_behavior(self.gesture_state, mapping)
        if not self._has_fresh_consent():
            self._broadcast_consent_state(0)
            return
        self._broadcast_consent_state(1)

        altitude_target = behavior.get("altitude_target", ALTITUDE_FLOOR_M)
        lateral_bias = behavior.get("lateral_bias", LATERAL_HOME_M)

        alt_normalized = self._clamp(
            (altitude_target - ALTITUDE_FLOOR_M) / ALTITUDE_SCALE_M,
            ALTITUDE_INPUT_RANGE,
        )
        lat_normalized = self._clamp(
            (lateral_bias - LATERAL_HOME_M) / LATERAL_SCALE_M,
            LATERAL_INPUT_RANGE,
        )

        for name in self.crafts:
            self._apply_lateral(name, lat_normalized, source=source)
            self._apply_altitude(name, alt_normalized, source=source)

    def _broadcast_consent_state(self, consent_state: int) -> None:
        """Optionally echo consent state so dashboards can mirror it."""

        if consent_state == self._last_broadcast_consent:
            return
        # We intentionally broadcast locally; listeners can subscribe without
        # having to query ROS or tail logs.
        try:
            client = udp_client.SimpleUDPClient(
                "127.0.0.1", self.server.server_address[1]
            )
            client.send_message(CONSENT_STATE_BROADCAST, consent_state)
        except Exception:
            # Do not spam logs if the broadcast fails; consent gating should not
            # be blocked by telemetry plumbing.
            return
        self._last_broadcast_consent = consent_state

    # ------------------------------------------------------------------
    # OSC handlers — translate crowd gestures into CrazySwarm2 service calls.
    # ------------------------------------------------------------------
    def _handle_global_lat(self, addr: str, *vals) -> None:
        value = self._extract_first_value(vals)
        if value is None:
            self.get_logger().warning(
                "%s handler received non-numeric payload: %s", addr, vals
            )
            return
        clamped = self._clamp(value, LATERAL_INPUT_RANGE)
        self.gesture_state["lat"] = clamped
        self._apply_behavior_from_gestures(source="global-lat")

    def _handle_global_alt(self, addr: str, *vals) -> None:
        value = self._extract_first_value(vals)
        if value is None:
            self.get_logger().warning(
                "%s handler received non-numeric payload: %s", addr, vals
            )
            return
        clamped = self._clamp(value, ALTITUDE_INPUT_RANGE)
        self.gesture_state["alt"] = clamped
        self._apply_behavior_from_gestures(source="global-alt")

    def _handle_direct_command(
        self, craft: str, axis: str, addr: str, *vals
    ) -> None:
        value = self._extract_first_value(vals)
        if value is None:
            self.get_logger().warning(
                "%s handler received non-numeric payload: %s", addr, vals
            )
            return
        if axis == "lat":
            clamped = self._clamp(value, LATERAL_INPUT_RANGE)
            self._apply_lateral(craft, clamped, source="direct")
        elif axis == "alt":
            clamped = self._clamp(value, ALTITUDE_INPUT_RANGE)
            self._apply_altitude(craft, clamped, source="direct")

    def _handle_consent(self, addr: str, *vals) -> None:
        """Handle participation zone updates from the vision tracker.

        The Processing tracker publishes a binary participation/consent flag when
        someone steps into or out of the floor-based zone. We treat that as a
        first-class control signal that can gate motion or trigger recipe edges.
        """

        raw_value = self._extract_first_value(vals)
        if raw_value is None:
            self.get_logger().warning(
                "%s handler received non-numeric payload: %s", addr, vals
            )
            return

        with self._consent_lock:
            previous = self._consent_state
            consent_state = _binary_consent_state(raw_value, default=previous)
            self._consent_state = consent_state
            self._last_consent_at = time.monotonic()
        self.gesture_state["consent"] = float(consent_state)

        if consent_state != previous:
            edge = "ON" if consent_state else "OFF"
            self.get_logger().info(
                "[consent] %s – participation zone flip via %s", edge, addr
            )
            self._handle_consent_edge(previous, consent_state)
        else:
            self.get_logger().debug(
                "[consent] steady at %d via %s", consent_state, addr
            )

        # Re-apply behavior so drones can settle into idle posture immediately.
        self._apply_behavior_from_gestures(source="consent")

    def _handle_patch(self, addr: str, *vals) -> None:
        recipe = self._extract_recipe_name(vals)
        if recipe is None:
            self.get_logger().warning(
                "%s handler received empty or invalid recipe payload: %s",
                addr,
                vals,
            )
            return

        try:
            new_mapping = load_mapping(
                base_path=self.base_mapping_path,
                recipe_name=recipe,
                recipes_dir=self.recipes_dir,
            )
        except Exception as exc:
            self.get_logger().warning(
                "Failed to load recipe '%s' via OSC (%s): %s",
                recipe,
                addr,
                exc,
            )
            return

        with self._mapping_lock:
            self.mapping = new_mapping
            self.current_recipe_name = recipe

        self.get_logger().info(
            "Switched recipe to '%s' via OSC (%s)", recipe, addr
        )

    # ------------------------------------------------------------------
    # CrazySwarm service helpers
    # ------------------------------------------------------------------
    def _apply_lateral(
        self, craft: str, normalized: float, source: str
    ) -> None:
        if not self._has_fresh_consent():
            return
        state = self.crafts[craft]
        target = (
            LATERAL_HOME_M
            + state.formation_offset
            + normalized * LATERAL_SCALE_M
        )
        previous = state.current_lateral
        if not math.isclose(
            state.current_lateral, target, rel_tol=1e-3, abs_tol=1e-3
        ):
            state.current_lateral = target
            self._enforce_collision_envelope(craft, previous)
            self.get_logger().info(
                "[%s] Lateral %.3f → %.3fm (%s)",
                craft,
                normalized,
                state.current_lateral,
                source,
            )
        self._send_go_to(state)

    def _apply_altitude(
        self, craft: str, normalized: float, source: str
    ) -> None:
        if not self._has_fresh_consent():
            return
        state = self.crafts[craft]
        height = ALTITUDE_FLOOR_M + normalized * ALTITUDE_SCALE_M
        if not math.isclose(
            state.current_altitude, height, rel_tol=1e-3, abs_tol=1e-3
        ):
            state.current_altitude = height
            self.get_logger().info(
                "[%s] Altitude %.3f → %.3fm (%s)",
                craft,
                normalized,
                height,
                source,
            )
        self._send_takeoff(state)

    def _send_go_to(self, state: CraftState) -> None:
        if not self._has_fresh_consent():
            return
        if not self._wait_for_service(
            state.go_to_client, f"{state.config.name}/go_to"
        ):
            return
        req = GoTo.Request()
        goal = None
        if hasattr(req, "goal"):
            goal = req.goal
        elif hasattr(req, "target"):
            goal = req.target
        if goal is not None:
            if LATERAL_AXIS.lower() == "x":
                goal.x = state.current_lateral
                goal.y = LATERAL_HOME_M
            else:
                goal.y = state.current_lateral
                goal.x = LATERAL_HOME_M
            goal.z = state.current_altitude
        if hasattr(req, "yaw"):
            req.yaw = 0.0
        self._apply_duration(req, GO_TO_DURATION_S)
        if hasattr(req, "relative"):
            req.relative = RELATIVE_MOVES
        if hasattr(req, "group_mask"):
            req.group_mask = GROUP_MASK
        self._send_live_request(state.go_to_client, req)

    def _send_takeoff(self, state: CraftState) -> None:
        if not self._has_fresh_consent():
            return
        if not self._wait_for_service(
            state.takeoff_client, f"{state.config.name}/takeoff"
        ):
            return
        req = Takeoff.Request()
        applied = False
        if hasattr(req, "height"):
            req.height = state.current_altitude
            applied = True
        if not applied and hasattr(req, "goal"):
            req.goal.z = state.current_altitude
        self._apply_duration(req, TAKEOFF_DURATION_S)
        if hasattr(req, "group_mask"):
            req.group_mask = GROUP_MASK
        self._send_live_request(state.takeoff_client, req)

    def _send_live_request(self, client, request) -> bool:
        """Serialize live service submission against consent-off transitions."""

        self._expire_stale_consent()
        with self._consent_lock:
            if self._consent_state != 1:
                return False
            client.call_async(request)
            return True

    def _land_and_stop_all(self, reason: str) -> None:
        self.get_logger().warning(
            "[consent] %s; landing and stopping all craft", reason
        )
        for state in self.crafts.values():
            state.current_altitude = 0.0
            self._send_land(state)
            self._send_stop(state)

    def _send_land(self, state: CraftState) -> None:
        if state.land_client is None or not self._wait_for_service(
            state.land_client, f"{state.config.name}/land"
        ):
            return
        req = Land.Request()
        if hasattr(req, "height"):
            req.height = 0.0
        self._apply_duration(req, LAND_DURATION_S)
        if hasattr(req, "group_mask"):
            req.group_mask = GROUP_MASK
        state.land_client.call_async(req)

    def _send_stop(self, state: CraftState) -> None:
        if state.stop_client is None or not self._wait_for_service(
            state.stop_client, f"{state.config.name}/stop"
        ):
            return
        req = Stop.Request()
        if hasattr(req, "group_mask"):
            req.group_mask = GROUP_MASK
        state.stop_client.call_async(req)

    @staticmethod
    def _apply_duration(request, seconds: float) -> None:
        if not hasattr(request, "duration"):
            return
        duration = request.duration
        if hasattr(duration, "sec") and hasattr(duration, "nanosec"):
            whole_seconds = int(seconds)
            duration.sec = whole_seconds
            duration.nanosec = int((seconds - whole_seconds) * 1_000_000_000)
        else:
            request.duration = float(seconds)

    # ------------------------------------------------------------------
    # Telemetry & logging helpers
    # ------------------------------------------------------------------
    def _wire_telemetry(self, cfg: CraftConfig, qos: QoSProfile) -> None:
        if get_message is None:
            return
        try:
            msg_type = (
                get_message(cfg.telemetry_type) if cfg.telemetry_type else None
            )
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            self.get_logger().warning(
                "[%s] Unable to import telemetry type %s: %s",
                cfg.name,
                cfg.telemetry_type,
                exc,
            )
            return
        if msg_type is None:
            return
        callback = partial(self._handle_telemetry, cfg.name)
        self.create_subscription(msg_type, cfg.telemetry_topic, callback, qos)
        self.get_logger().info(
            "[%s] Telemetry subscription hooked to %s (%s)",
            cfg.name,
            cfg.telemetry_topic,
            cfg.telemetry_type,
        )

    def _handle_telemetry(self, craft: str, msg) -> None:
        if message_to_ordereddict is None:
            data = {"repr": repr(msg)}
        else:
            try:
                data = message_to_ordereddict(msg)
            except Exception:  # pragma: no cover - defensive fallback
                data = {"repr": repr(msg)}
        self.crafts[craft].telemetry = data

    def _report_status(self) -> None:
        chunks: List[str] = []
        for name, state in self.crafts.items():
            chunk = f"{name}: alt={state.current_altitude:.2f}m lat={state.current_lateral:.2f}m"
            if state.telemetry:
                chunk += f" telem={self._summarize_telemetry(state.telemetry)}"
            chunks.append(chunk)
        min_dist, pair = self._fleet_min_distance()
        if math.isfinite(min_dist):
            chunks.append(f"min_sep={min_dist:.2f}m pair={pair[0]}-{pair[1]}")
        snapshot = " | ".join(chunks)
        if snapshot != self._last_status_snapshot:
            self.get_logger().info("Fleet snapshot → %s", snapshot)
            self._last_status_snapshot = snapshot

    def _fleet_min_distance(self) -> Tuple[float, Tuple[str, str]]:
        names = list(self.crafts.keys())
        min_dist = math.inf
        min_pair = ("-", "-")
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a = self.crafts[names[i]]
                b = self.crafts[names[j]]
                if LATERAL_AXIS.lower() == "x":
                    ax, ay = a.current_lateral, LATERAL_HOME_M
                    bx, by = b.current_lateral, LATERAL_HOME_M
                else:
                    ax, ay = LATERAL_HOME_M, a.current_lateral
                    bx, by = LATERAL_HOME_M, b.current_lateral
                dz = b.current_altitude - a.current_altitude
                dx = bx - ax
                dy = by - ay
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist < min_dist:
                    min_dist = dist
                    min_pair = (a.config.name, b.config.name)
        return min_dist, min_pair

    def _enforce_collision_envelope(
        self, craft: str, previous_lateral: float
    ) -> None:
        min_dist, pair = self._fleet_min_distance()
        if min_dist >= self._min_separation_m:
            return
        self.crafts[craft].current_lateral = previous_lateral
        now = time.time()
        if now - self._last_collision_warn_at >= 1.0:
            self.get_logger().warning(
                "[guard] rejected lateral command for %s: pair %s-%s would dip below %.2fm (%.2fm)",
                craft,
                pair[0],
                pair[1],
                self._min_separation_m,
                min_dist,
            )
            self._last_collision_warn_at = now

    @staticmethod
    def _summarize_telemetry(data: Dict[str, object]) -> str:
        if not isinstance(data, dict):
            return str(data)
        items = []
        for key, value in list(data.items())[:3]:
            if isinstance(value, dict):
                items.append(f"{key}=…")
            else:
                items.append(f"{key}={value}")
        return ", ".join(items) if items else "(empty)"

    # ------------------------------------------------------------------
    # Utility helpers lifted from the single-drone prototype
    # ------------------------------------------------------------------
    def _wait_for_service(self, client, label: str) -> bool:
        if client.wait_for_service(timeout_sec=0.0):
            return True
        self.get_logger().warning(
            "%s service not ready; dropping command", label
        )
        return False

    @staticmethod
    def _extract_first_value(vals: Iterable[float]) -> Optional[float]:
        if not vals:
            return None
        try:
            return float(vals[0])
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _extract_recipe_name(vals: Iterable[Any]) -> Optional[str]:
        if not vals:
            return None
        raw = vals[0]
        if raw is None:
            return None
        if isinstance(raw, bytes):
            try:
                raw = raw.decode("utf-8")
            except Exception:
                return None
        text = str(raw).strip()
        return text or None

    @staticmethod
    def _clamp(value: float, bounds: Iterable[float]) -> float:
        lower, upper = bounds
        return max(lower, min(upper, value))

    def stop_osc_server(self) -> None:
        if hasattr(self, "server"):
            try:
                self.server.shutdown()
                self.server.server_close()
            except Exception:  # pragma: no cover - best effort cleanup
                pass

    def destroy_node(self) -> None:  # type: ignore[override]
        self.stop_osc_server()
        super().destroy_node()


def _parse_cli(
    argv: Optional[List[str]] = None,
) -> Tuple[argparse.Namespace, List[str]]:
    parser = argparse.ArgumentParser(
        description="OSC-driven CrazySwarm2 bridge"
    )
    parser.add_argument(
        "--recipe",
        help="Optional mapping recipe to apply at startup",
        default=None,
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Enable virtual swarm simulation",
    )
    parser.add_argument(
        "--bind",
        default=OSC_BIND_ADDRESS,
        help=(
            "OSC listen address (default: 127.0.0.1). Use 0.0.0.0 only "
            "on a physically isolated, trusted control network."
        ),
    )
    parser.add_argument(
        "--osc-port",
        type=int,
        default=OSC_BIND_PORT,
        help="OSC control listen port",
    )
    parser.add_argument(
        "--consent-timeout",
        type=float,
        default=DEFAULT_CONSENT_TIMEOUT_S,
        help="Seconds without a consent heartbeat before forced OFF",
    )
    parser.add_argument(
        "--sim-drones",
        type=int,
        default=4,
        help="Number of virtual drones to simulate",
    )
    parser.add_argument(
        "--sim-update-rate",
        type=float,
        default=30.0,
        help="Frequency in Hz for simulated drone updates",
    )
    parser.add_argument(
        "--sim-osc-target",
        default=SIM_OSC_TARGET,
        help="OSC host to publish simulated drone state",
    )
    parser.add_argument(
        "--sim-osc-port",
        type=int,
        default=SIM_OSC_PORT,
        help="OSC port to publish simulated drone state",
    )
    parser.add_argument(
        "--sim-min-separation",
        type=float,
        default=SIM_MIN_SEPARATION_M,
        help="Minimum allowed distance in meters between simulated drones.",
    )
    return parser.parse_known_args(argv)


def _initialize_mapping(
    recipe_name: Optional[str],
) -> Tuple[Dict[str, Any], Optional[str]]:
    """Load the base mapping and optionally an overlay recipe, with safe fallback."""

    if recipe_name is None:
        mapping = load_mapping(
            base_path=DEFAULT_BASE_MAPPING_PATH,
            recipes_dir=DEFAULT_RECIPES_DIR,
        )
        print(f"Loaded base mapping from {DEFAULT_BASE_MAPPING_PATH}")
        print("No recipe specified; using base mapping only")
        return mapping, None

    try:
        mapping = load_mapping(
            base_path=DEFAULT_BASE_MAPPING_PATH,
            recipe_name=recipe_name,
            recipes_dir=DEFAULT_RECIPES_DIR,
        )
    except Exception as exc:
        print(
            f"Failed to load recipe '{recipe_name}'; falling back to base mapping ({exc})",
            file=sys.stderr,
        )
        mapping = load_mapping(
            base_path=DEFAULT_BASE_MAPPING_PATH,
            recipes_dir=DEFAULT_RECIPES_DIR,
        )
        print(f"Loaded base mapping from {DEFAULT_BASE_MAPPING_PATH}")
        print("Using base mapping only after recipe load failure")
        return mapping, None

    print(f"Loaded base mapping from {DEFAULT_BASE_MAPPING_PATH}")
    print(
        f"Applied recipe '{recipe_name}' from {DEFAULT_RECIPES_DIR}/{recipe_name}.yaml"
    )
    return mapping, recipe_name


def _map_gestures_to_behavior(
    gestures: Dict[str, float], mapping: Dict[str, Any]
) -> Dict[str, float]:
    """Translate raw gesture streams into a simplified behavior dictionary.

    Consent travels alongside the rest of the OSC stream. Treat it like any
    other control so mapping.yaml (and recipes) can shape how strictly we gate
    motion when nobody is in the participation zone.
    """

    lat = SwarmNode._clamp(gestures.get("lat", 0.0), LATERAL_INPUT_RANGE)
    alt = SwarmNode._clamp(gestures.get("alt", 0.0), ALTITUDE_INPUT_RANGE)
    yaw = SwarmNode._clamp(gestures.get("yaw", 0.0), (-1.0, 1.0))
    crowd = gestures.get("crowd", 0.0) or 0.0

    consent_cfg = (
        mapping.get("consent", {}) if isinstance(mapping, dict) else {}
    )
    consent_default = consent_cfg.get("default_state", 0)
    consent_raw = gestures.get("consent", consent_default)
    consent = max(0.0, min(1.0, consent_raw))

    # Convert normalized controls to meters and small angular nudges.
    altitude_target = ALTITUDE_FLOOR_M + alt * ALTITUDE_SCALE_M
    lateral_bias = LATERAL_HOME_M + lat * LATERAL_SCALE_M
    yaw_bias = yaw * (math.pi / 2.0)
    jitter = abs(crowd) * 0.5

    gate_motion = consent_cfg.get("gate_motion", True)
    mode = (consent_cfg.get("mode") or "binary").lower()
    idle_alt = consent_cfg.get("idle_altitude", ALTITUDE_INPUT_RANGE[0])
    idle_jitter = consent_cfg.get("idle_jitter", 0.0)

    if gate_motion:
        if mode == "smooth":
            lateral_bias *= consent
            yaw_bias *= consent
            jitter *= consent
        else:  # default/binary gate
            if consent < 0.5:
                altitude_target = (
                    ALTITUDE_FLOOR_M + idle_alt * ALTITUDE_SCALE_M
                )
                lateral_bias = LATERAL_HOME_M
                yaw_bias = 0.0
                jitter = idle_jitter

    return {
        "altitude_target": altitude_target,
        "lateral_bias": lateral_bias,
        "yaw_bias": yaw_bias,
        "jitter": jitter,
        "consent": consent,
        "recipe": mapping,
    }


def _run_simulation(
    parsed: argparse.Namespace,
    mapping: Dict[str, Any],
    recipe_name: Optional[str],
) -> None:
    gesture_state: Dict[str, float] = {}
    mapping_lock = threading.Lock()
    consent_lock = threading.Lock()
    active_mapping = mapping
    active_recipe = recipe_name
    consent_cfg = (
        mapping.get("consent", {}) if isinstance(mapping, dict) else {}
    )
    consent_state = _binary_consent_state(consent_cfg.get("default_state", 0))
    gesture_state["consent"] = float(consent_state)
    last_consent_at: Optional[float] = None
    last_sent_consent = consent_state
    bench_seq = 0
    pending_bench: Optional[Tuple[int, float]] = None
    bench_clock_start = time.monotonic()
    disp = dispatcher.Dispatcher()

    def _mk_handler(label: str):
        def _handler(addr: str, *vals) -> None:
            value = SwarmNode._extract_first_value(vals)
            if value is None:
                return
            gesture_state[label] = value

        return _handler

    def _handle_sim_consent(addr: str, *vals) -> None:
        nonlocal consent_state, active_mapping, active_recipe, last_consent_at
        value = SwarmNode._extract_first_value(vals)
        if value is None:
            return
        new_state = _binary_consent_state(value, default=consent_state)
        with consent_lock:
            previous_state = consent_state
            consent_state = new_state
            last_consent_at = time.monotonic()
            gesture_state["consent"] = float(new_state)
        if new_state == previous_state:
            return

        print(
            f"[consent] {'ON' if consent_state else 'OFF'} – participation zone flip via {addr}"
        )

        with mapping_lock:
            cfg = (
                active_mapping.get("consent", {})
                if isinstance(active_mapping, dict)
                else {}
            )
        if cfg.get("auto_recipes", False):
            recipe_key = "on_recipe" if consent_state else "off_recipe"
            recipe_target = (cfg.get(recipe_key) or "").strip()
            if recipe_target and recipe_target.lower() != "none":
                try:
                    updated_mapping = load_mapping(
                        base_path=DEFAULT_BASE_MAPPING_PATH,
                        recipe_name=recipe_target,
                        recipes_dir=DEFAULT_RECIPES_DIR,
                    )
                except Exception as exc:
                    edge_label = "ON" if consent_state else "OFF"
                    print(
                        f"Auto-recipe: failed to load '{recipe_target}' "
                        f"on consent {edge_label} ({exc})",
                        file=sys.stderr,
                    )
                else:
                    with mapping_lock:
                        active_mapping = updated_mapping
                        active_recipe = recipe_target
                    edge_label = "ON" if consent_state else "OFF"
                    print(
                        f"Auto-recipe: switched to '{recipe_target}' "
                        f"on consent {edge_label}"
                    )

    disp.map(GLOBAL_OSC_ROUTES["lat"], _mk_handler("lat"))
    disp.map(GLOBAL_OSC_ROUTES["alt"], _mk_handler("alt"))
    disp.map("/pd/yaw", _mk_handler("yaw"))
    disp.map("/pd/crowd", _mk_handler("crowd"))
    disp.map("/pd/consent", _handle_sim_consent)

    def _handle_bench_ping(addr: str, *vals) -> None:
        nonlocal bench_seq, pending_bench
        now_rel = time.monotonic() - bench_clock_start
        seq = bench_seq
        if vals:
            maybe_seq = SwarmNode._extract_first_value(vals[:1])
            if maybe_seq is not None:
                seq = int(maybe_seq)
        bench_seq = max(bench_seq + 1, seq + 1)
        pending_bench = (seq, now_rel)

    disp.map(SIM_BENCH_PING_ROUTE, _handle_bench_ping)

    _warn_if_exposed_bind(parsed.bind, parsed.osc_port)
    server = osc_server.ThreadingOSCUDPServer(
        (parsed.bind, parsed.osc_port), disp
    )
    osc_thread = threading.Thread(target=server.serve_forever, daemon=True)
    osc_thread.start()

    target = parsed.sim_osc_target
    port = parsed.sim_osc_port
    client = udp_client.SimpleUDPClient(target, port)

    drones = [VirtualDrone(i) for i in range(max(1, parsed.sim_drones))]
    update_rate = (
        parsed.sim_update_rate if parsed.sim_update_rate > 0 else 30.0
    )
    dt = 1.0 / update_rate

    print(
        f"Simulation mode: ON ({len(drones)} virtual drones @ {update_rate} Hz)"
    )
    print(f"Sim OSC control listener: {parsed.bind}:{parsed.osc_port}")
    print(
        "Consent heartbeat timeout: "
        f"{max(0.1, parsed.consent_timeout):.2f}s"
    )
    print(f"Sim OSC publish target: {target}:{port}")
    print(
        f"Collision guard: min separation {max(parsed.sim_min_separation, 0.0):.2f}m"
    )
    print(
        f"Benchmark ping route: {SIM_BENCH_PING_ROUTE} "
        f"(ack on {SIM_BENCH_ACK_ROUTE})"
    )

    last_summary = time.time()
    try:
        while True:
            loop_start = time.time()
            consent_expired = False
            with consent_lock:
                if consent_state == 1 and (
                    last_consent_at is None
                    or time.monotonic() - last_consent_at
                    > max(0.1, parsed.consent_timeout)
                ):
                    consent_state = 0
                    last_consent_at = None
                    gesture_state["consent"] = 0.0
                    consent_expired = True
            if consent_expired:
                print(
                    "[consent] tracker heartbeat stale; forcing simulation OFF",
                    file=sys.stderr,
                )
            with mapping_lock:
                local_mapping = active_mapping
                local_recipe = active_recipe
            behavior = _map_gestures_to_behavior(gesture_state, local_mapping)
            for drone in drones:
                drone.update(dt, behavior)
            adjustments = enforce_min_separation(
                drones, max(parsed.sim_min_separation, 0.0)
            )
            min_dist = minimum_pairwise_distance(drones)

            if adjustments > 0:
                client.send_message(
                    SIM_COLLISION_ROUTE,
                    [float(min_dist), int(adjustments)],
                )

            if pending_bench is not None:
                seq, recv_rel = pending_bench
                pending_bench = None
                emit_rel = time.monotonic() - bench_clock_start
                client.send_message(
                    SIM_BENCH_ACK_ROUTE,
                    [int(seq), float(recv_rel), float(emit_rel)],
                )

            for drone in drones:
                client.send_message(
                    f"/pd/sim/drone/{drone.drone_id}/pos",
                    [drone.x, drone.y, drone.z],
                )
                client.send_message(
                    f"/pd/sim/drone/{drone.drone_id}/yaw", drone.yaw
                )
            if consent_state != last_sent_consent:
                client.send_message("/pd/sim/consent", consent_state)
                last_sent_consent = consent_state

            now = time.time()
            if now - last_summary >= 1.0:
                recipe_label = local_recipe or "base"
                client.send_message(
                    "/pd/sim/summary", [len(drones), recipe_label]
                )
                head = drones[0]
                print(
                    f"[sim] drone 0 → x={head.x:.2f} y={head.y:.2f} "
                    f"z={head.z:.2f} yaw={head.yaw:.2f} "
                    f"(recipe={recipe_label})"
                )
                last_summary = now

            elapsed = time.time() - loop_start
            sleep_for = max(0.0, dt - elapsed)
            time.sleep(sleep_for)
    except KeyboardInterrupt:
        print("Simulation interrupted by user; shutting down")
    finally:
        try:
            server.shutdown()
            server.server_close()
        except Exception:
            pass


def main(argv: Optional[List[str]] = None) -> None:
    parsed, remaining = _parse_cli(argv)
    mapping, recipe_name = _initialize_mapping(parsed.recipe)

    if parsed.simulate:
        _run_simulation(parsed, mapping, recipe_name)
        return

    if rclpy is None:
        raise RuntimeError(
            "Real swarm mode requires ROS2 + CrazySwarm2 dependencies. "
            f"Import failure: {ROS_IMPORT_ERROR}"
        )

    print("Simulation mode: OFF (real drone path)")

    rclpy.init(args=remaining)
    node = SwarmNode(
        mapping=mapping,
        recipe_name=recipe_name,
        base_mapping_path=DEFAULT_BASE_MAPPING_PATH,
        recipes_dir=DEFAULT_RECIPES_DIR,
        osc_bind=parsed.bind,
        osc_port=parsed.osc_port,
        consent_timeout_s=parsed.consent_timeout,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        node.stop_osc_server()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
