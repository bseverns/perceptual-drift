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

import rclpy
from pythonosc import dispatcher, osc_server, udp_client
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from crazyflie_interfaces.srv import GoTo, Takeoff
from software.swarm.mapping_loader import load_mapping
from software.swarm.virtual_swarm import VirtualDrone

# ---------------------------------------------------------------------------
# Fleet layout + OSC wiring — tweak these to match your venue rig.
# ---------------------------------------------------------------------------

OSC_BIND_ADDRESS = "0.0.0.0"
OSC_BIND_PORT = 9010

DEFAULT_BASE_MAPPING_PATH = "config/mapping.yaml"
DEFAULT_RECIPES_DIR = "config/recipes"
SIM_OSC_TARGET = "127.0.0.1"
SIM_OSC_PORT = 9100
CONSENT_STATE_BROADCAST = "/pd/current_consent"

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
    telemetry_topic: Optional[str] = None
    telemetry_type: Optional[str] = None


DEFAULT_FLEET: List[CraftConfig] = [
    CraftConfig(
        name="cf1",
        go_to_service="/cf1/go_to",
        takeoff_service="/cf1/takeoff",
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

# Motion timing and default metadata.  Set RELATIVE_MOVES to True if your GoTo
# service expects relative offsets instead of absolute coordinates.
GO_TO_DURATION_S = 0.5
TAKEOFF_DURATION_S = 1.5
GROUP_MASK = 0
RELATIVE_MOVES = False

# Telemetry snapshots print every few seconds so operators can see the roundtrip.
STATUS_REPORT_INTERVAL_S = 5.0


@dataclass
class CraftState:
    config: CraftConfig
    go_to_client: Any = field(repr=False)
    takeoff_client: Any = field(repr=False)
    current_altitude: float = ALTITUDE_FLOOR_M
    current_lateral: float = LATERAL_HOME_M
    formation_offset: float = 0.0
    telemetry: Optional[Dict[str, object]] = None


class SwarmNode(Node):
    """ROS 2 node that listens for OSC and relays to a CrazySwarm2 fleet."""

    def __init__(
        self,
        fleet: Optional[List[CraftConfig]] = None,
        mapping: Optional[Dict[str, Any]] = None,
        recipe_name: Optional[str] = None,
        base_mapping_path: str = DEFAULT_BASE_MAPPING_PATH,
        recipes_dir: str = DEFAULT_RECIPES_DIR,
    ):
        super().__init__("swarm_node")
        self.fleet_cfg = fleet or DEFAULT_FLEET
        if not self.fleet_cfg:
            raise RuntimeError("At least one CraftConfig is required to run the swarm bridge")

        self.base_mapping_path = base_mapping_path
        self.recipes_dir = recipes_dir
        self.mapping = mapping or load_mapping(base_path=self.base_mapping_path, recipes_dir=self.recipes_dir)
        self.current_recipe_name = recipe_name
        self._mapping_lock = threading.Lock()
        self._consent_state = self._get_default_consent_state(self.mapping)
        self._last_consent_at: Optional[float] = None
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
        self.gesture_state: Dict[str, float] = {"consent": float(self._consent_state)}

        span = max(len(self.fleet_cfg) - 1, 0)
        for index, cfg in enumerate(self.fleet_cfg):
            go_to_client = self.create_client(GoTo, cfg.go_to_service)
            takeoff_client = self.create_client(Takeoff, cfg.takeoff_service)
            formation_offset = (index - span / 2.0) * 0.6
            self.crafts[cfg.name] = CraftState(
                config=cfg,
                go_to_client=go_to_client,
                takeoff_client=takeoff_client,
                formation_offset=formation_offset,
            )
            for axis, address in cfg.osc_addresses.items():
                if axis not in {"lat", "alt"}:
                    continue
                handler = partial(self._handle_direct_command, cfg.name, axis)
                self.dispatcher.map(address, handler)
            if cfg.telemetry_topic and cfg.telemetry_type:
                self._wire_telemetry(cfg, qos)

        self.server = osc_server.ThreadingOSCUDPServer(
            (OSC_BIND_ADDRESS, OSC_BIND_PORT),
            self.dispatcher,
        )
        self._osc_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self._osc_thread.start()
        self.get_logger().info(
            "OSC listening on %s:%d for %s",
            *self.server.server_address,
            ", ".join(self.crafts.keys()),
        )

        self.create_timer(STATUS_REPORT_INTERVAL_S, self._report_status)

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
        return 1 if cfg.get("default_state", 1) else 0

    def _handle_consent_edge(self, previous: int, current: int) -> None:
        """React to consent transitions: logging, auto-recipes, idling."""

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
                except Exception as exc:  # pragma: no cover - defensive edge logging
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
        consent_state = 1 if behavior.get("consent", 1.0) >= 0.5 else 0
        if consent_state != self._consent_state:
            self._handle_consent_edge(self._consent_state, consent_state)
        self._consent_state = consent_state
        self._broadcast_consent_state(consent_state)

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
            client = udp_client.SimpleUDPClient("127.0.0.1", self.server.server_address[1])
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
            self.get_logger().warning("%s handler received non-numeric payload: %s", addr, vals)
            return
        clamped = self._clamp(value, LATERAL_INPUT_RANGE)
        self.gesture_state["lat"] = clamped
        self._apply_behavior_from_gestures(source="global-lat")

    def _handle_global_alt(self, addr: str, *vals) -> None:
        value = self._extract_first_value(vals)
        if value is None:
            self.get_logger().warning("%s handler received non-numeric payload: %s", addr, vals)
            return
        clamped = self._clamp(value, ALTITUDE_INPUT_RANGE)
        self.gesture_state["alt"] = clamped
        self._apply_behavior_from_gestures(source="global-alt")

    def _handle_direct_command(self, craft: str, axis: str, addr: str, *vals) -> None:
        value = self._extract_first_value(vals)
        if value is None:
            self.get_logger().warning("%s handler received non-numeric payload: %s", addr, vals)
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
            self.get_logger().warning("%s handler received non-numeric payload: %s", addr, vals)
            return

        previous = self._consent_state
        consent_state = 1 if raw_value >= 1.0 else 0
        self._consent_state = consent_state
        self._last_consent_at = time.time()
        self.gesture_state["consent"] = float(consent_state)

        if consent_state != previous:
            edge = "ON" if consent_state else "OFF"
            self.get_logger().info("[consent] %s – participation zone flip via %s", edge, addr)
            self._handle_consent_edge(previous, consent_state)
        else:
            self.get_logger().debug("[consent] steady at %d via %s", consent_state, addr)

        # Re-apply behavior so drones can settle into idle posture immediately.
        self._apply_behavior_from_gestures(source="consent")

    def _handle_patch(self, addr: str, *vals) -> None:
        recipe = self._extract_recipe_name(vals)
        if recipe is None:
            self.get_logger().warning("%s handler received empty or invalid recipe payload: %s", addr, vals)
            return

        try:
            new_mapping = load_mapping(
                base_path=self.base_mapping_path,
                recipe_name=recipe,
                recipes_dir=self.recipes_dir,
            )
        except Exception as exc:
            self.get_logger().warning("Failed to load recipe '%s' via OSC (%s): %s", recipe, addr, exc)
            return

        with self._mapping_lock:
            self.mapping = new_mapping
            self.current_recipe_name = recipe

        self.get_logger().info("Switched recipe to '%s' via OSC (%s)", recipe, addr)

    # ------------------------------------------------------------------
    # CrazySwarm service helpers
    # ------------------------------------------------------------------
    def _apply_lateral(self, craft: str, normalized: float, source: str) -> None:
        state = self.crafts[craft]
        consent_cfg = self._get_consent_config()
        if consent_cfg.get("gate_motion", True) and self._consent_state == 0:
            normalized = 0.0
        target = LATERAL_HOME_M + state.formation_offset + normalized * LATERAL_SCALE_M
        if not math.isclose(state.current_lateral, target, rel_tol=1e-3, abs_tol=1e-3):
            state.current_lateral = target
            self.get_logger().info(
                "[%s] Lateral %.3f → %.3fm (%s)",
                craft,
                normalized,
                target,
                source,
            )
        self._send_go_to(state)

    def _apply_altitude(self, craft: str, normalized: float, source: str) -> None:
        state = self.crafts[craft]
        consent_cfg = self._get_consent_config()
        if consent_cfg.get("gate_motion", True) and self._consent_state == 0:
            normalized = consent_cfg.get("idle_altitude", ALTITUDE_INPUT_RANGE[0])
        height = ALTITUDE_FLOOR_M + normalized * ALTITUDE_SCALE_M
        if not math.isclose(state.current_altitude, height, rel_tol=1e-3, abs_tol=1e-3):
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
        if not self._wait_for_service(state.go_to_client, f"{state.config.name}/go_to"):
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
        if hasattr(req, "duration"):
            req.duration = GO_TO_DURATION_S
        if hasattr(req, "relative"):
            req.relative = RELATIVE_MOVES
        if hasattr(req, "group_mask"):
            req.group_mask = GROUP_MASK
        state.go_to_client.call_async(req)

    def _send_takeoff(self, state: CraftState) -> None:
        if not self._wait_for_service(state.takeoff_client, f"{state.config.name}/takeoff"):
            return
        req = Takeoff.Request()
        applied = False
        if hasattr(req, "height"):
            req.height = state.current_altitude
            applied = True
        if not applied and hasattr(req, "goal"):
            req.goal.z = state.current_altitude
        if hasattr(req, "duration"):
            req.duration = TAKEOFF_DURATION_S
        if hasattr(req, "group_mask"):
            req.group_mask = GROUP_MASK
        state.takeoff_client.call_async(req)

    # ------------------------------------------------------------------
    # Telemetry & logging helpers
    # ------------------------------------------------------------------
    def _wire_telemetry(self, cfg: CraftConfig, qos: QoSProfile) -> None:
        try:
            msg_type = get_message(cfg.telemetry_type) if cfg.telemetry_type else None
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
        snapshot = " | ".join(chunks)
        if snapshot != self._last_status_snapshot:
            self.get_logger().info("Fleet snapshot → %s", snapshot)
            self._last_status_snapshot = snapshot

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
        self.get_logger().warning("%s service not ready; dropping command", label)
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


def _parse_cli(argv: Optional[List[str]] = None) -> Tuple[argparse.Namespace, List[str]]:
    parser = argparse.ArgumentParser(description="OSC-driven CrazySwarm2 bridge")
    parser.add_argument("--recipe", help="Optional mapping recipe to apply at startup", default=None)
    parser.add_argument("--simulate", action="store_true", help="Enable virtual swarm simulation")
    parser.add_argument("--sim-drones", type=int, default=4, help="Number of virtual drones to simulate")
    parser.add_argument(
        "--sim-update-rate",
        type=float,
        default=30.0,
        help="Frequency in Hz for simulated drone updates",
    )
    parser.add_argument(
        "--sim-osc-target", default=SIM_OSC_TARGET, help="OSC host to publish simulated drone state"
    )
    parser.add_argument(
        "--sim-osc-port", type=int, default=SIM_OSC_PORT, help="OSC port to publish simulated drone state"
    )
    return parser.parse_known_args(argv)


def _initialize_mapping(recipe_name: Optional[str]) -> Tuple[Dict[str, Any], Optional[str]]:
    """Load the base mapping and optionally an overlay recipe, with safe fallback."""

    if recipe_name is None:
        mapping = load_mapping(base_path=DEFAULT_BASE_MAPPING_PATH, recipes_dir=DEFAULT_RECIPES_DIR)
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
        mapping = load_mapping(base_path=DEFAULT_BASE_MAPPING_PATH, recipes_dir=DEFAULT_RECIPES_DIR)
        print(f"Loaded base mapping from {DEFAULT_BASE_MAPPING_PATH}")
        print("Using base mapping only after recipe load failure")
        return mapping, None

    print(f"Loaded base mapping from {DEFAULT_BASE_MAPPING_PATH}")
    print(f"Applied recipe '{recipe_name}' from {DEFAULT_RECIPES_DIR}/{recipe_name}.yaml")
    return mapping, recipe_name


def _map_gestures_to_behavior(gestures: Dict[str, float], mapping: Dict[str, Any]) -> Dict[str, float]:
    """Translate raw gesture streams into a simplified behavior dictionary.

    Consent travels alongside the rest of the OSC stream. Treat it like any
    other control so mapping.yaml (and recipes) can shape how strictly we gate
    motion when nobody is in the participation zone.
    """

    lat = SwarmNode._clamp(gestures.get("lat", 0.0), LATERAL_INPUT_RANGE)
    alt = SwarmNode._clamp(gestures.get("alt", 0.0), ALTITUDE_INPUT_RANGE)
    yaw = SwarmNode._clamp(gestures.get("yaw", 0.0), (-1.0, 1.0))
    crowd = gestures.get("crowd", 0.0) or 0.0

    consent_cfg = mapping.get("consent", {}) if isinstance(mapping, dict) else {}
    consent_default = consent_cfg.get("default_state", 1)
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
                altitude_target = ALTITUDE_FLOOR_M + idle_alt * ALTITUDE_SCALE_M
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


def _run_simulation(parsed: argparse.Namespace, mapping: Dict[str, Any], recipe_name: Optional[str]) -> None:
    gesture_state: Dict[str, float] = {}
    mapping_lock = threading.Lock()
    active_mapping = mapping
    active_recipe = recipe_name
    consent_cfg = mapping.get("consent", {}) if isinstance(mapping, dict) else {}
    consent_state = 1 if (consent_cfg.get("default_state", 1) or 0) else 0
    gesture_state["consent"] = float(consent_state)
    last_sent_consent = consent_state
    disp = dispatcher.Dispatcher()

    def _mk_handler(label: str):
        def _handler(addr: str, *vals) -> None:
            value = SwarmNode._extract_first_value(vals)
            if value is None:
                return
            gesture_state[label] = value

        return _handler

    def _handle_sim_consent(addr: str, *vals) -> None:
        nonlocal consent_state, active_mapping, active_recipe
        value = SwarmNode._extract_first_value(vals)
        if value is None:
            return
        new_state = 1 if value >= 1.0 else 0
        gesture_state["consent"] = float(new_state)
        if new_state == consent_state:
            return

        consent_state = new_state
        print(f"[consent] {'ON' if consent_state else 'OFF'} – participation zone flip via {addr}")

        with mapping_lock:
            cfg = active_mapping.get("consent", {}) if isinstance(active_mapping, dict) else {}
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
                    print(
                        f"Auto-recipe: failed to load '{recipe_target}' on consent {'ON' if consent_state else 'OFF'} ({exc})",
                        file=sys.stderr,
                    )
                else:
                    with mapping_lock:
                        active_mapping = updated_mapping
                        active_recipe = recipe_target
                    print(
                        f"Auto-recipe: switched to '{recipe_target}' on consent {'ON' if consent_state else 'OFF'}"
                    )

    disp.map(GLOBAL_OSC_ROUTES["lat"], _mk_handler("lat"))
    disp.map(GLOBAL_OSC_ROUTES["alt"], _mk_handler("alt"))
    disp.map("/pd/yaw", _mk_handler("yaw"))
    disp.map("/pd/crowd", _mk_handler("crowd"))
    disp.map("/pd/consent", _handle_sim_consent)

    server = osc_server.ThreadingOSCUDPServer((OSC_BIND_ADDRESS, OSC_BIND_PORT), disp)
    osc_thread = threading.Thread(target=server.serve_forever, daemon=True)
    osc_thread.start()

    target = parsed.sim_osc_target
    port = parsed.sim_osc_port
    client = udp_client.SimpleUDPClient(target, port)

    drones = [VirtualDrone(i) for i in range(max(1, parsed.sim_drones))]
    update_rate = parsed.sim_update_rate if parsed.sim_update_rate > 0 else 30.0
    dt = 1.0 / update_rate

    print(f"Simulation mode: ON ({len(drones)} virtual drones @ {update_rate} Hz)")
    print(f"Sim OSC publish target: {target}:{port}")

    last_summary = time.time()
    try:
        while True:
            loop_start = time.time()
            with mapping_lock:
                local_mapping = active_mapping
                local_recipe = active_recipe
            behavior = _map_gestures_to_behavior(gesture_state, local_mapping)
            for drone in drones:
                drone.update(dt, behavior)
                client.send_message(f"/pd/sim/drone/{drone.drone_id}/pos", [drone.x, drone.y, drone.z])
                client.send_message(f"/pd/sim/drone/{drone.drone_id}/yaw", drone.yaw)
            if consent_state != last_sent_consent:
                client.send_message("/pd/sim/consent", consent_state)
                last_sent_consent = consent_state

            now = time.time()
            if now - last_summary >= 1.0:
                recipe_label = local_recipe or "base"
                client.send_message("/pd/sim/summary", [len(drones), recipe_label])
                head = drones[0]
                print(
                    f"[sim] drone 0 → x={head.x:.2f} y={head.y:.2f} z={head.z:.2f} yaw={head.yaw:.2f} (recipe={recipe_label})"
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

    print("Simulation mode: OFF (real drone path)")

    rclpy.init(args=remaining)
    node = SwarmNode(
        mapping=mapping,
        recipe_name=recipe_name,
        base_mapping_path=DEFAULT_BASE_MAPPING_PATH,
        recipes_dir=DEFAULT_RECIPES_DIR,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        node.stop_osc_server()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
