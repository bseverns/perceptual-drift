#!/usr/bin/env python3
"""OSC-driven CrazySwarm2 bridge that understands a full swarm, not just one whoop."""

import math
import threading
from dataclasses import dataclass, field
from functools import partial
from typing import Any, Dict, Iterable, List, Optional

import rclpy
from pythonosc import dispatcher, osc_server
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from crazyflie_interfaces.srv import GoTo, Takeoff

# ---------------------------------------------------------------------------
# Fleet layout + OSC wiring — tweak these to match your venue rig.
# ---------------------------------------------------------------------------

OSC_BIND_ADDRESS = "0.0.0.0"
OSC_BIND_PORT = 9010

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

    def __init__(self, fleet: Optional[List[CraftConfig]] = None):
        super().__init__("swarm_node")
        self.fleet_cfg = fleet or DEFAULT_FLEET
        if not self.fleet_cfg:
            raise RuntimeError("At least one CraftConfig is required to run the swarm bridge")

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10,
        )

        self.dispatcher = dispatcher.Dispatcher()
        self.dispatcher.map(GLOBAL_OSC_ROUTES["lat"], self._handle_global_lat)
        self.dispatcher.map(GLOBAL_OSC_ROUTES["alt"], self._handle_global_alt)

        self.crafts: Dict[str, CraftState] = {}
        self._last_status_snapshot: Optional[str] = None

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
    # OSC handlers — translate crowd gestures into CrazySwarm2 service calls.
    # ------------------------------------------------------------------
    def _handle_global_lat(self, addr: str, *vals) -> None:
        value = self._extract_first_value(vals)
        if value is None:
            self.get_logger().warning("%s handler received non-numeric payload: %s", addr, vals)
            return
        clamped = self._clamp(value, LATERAL_INPUT_RANGE)
        for name in self.crafts:
            self._apply_lateral(name, clamped, source="global")

    def _handle_global_alt(self, addr: str, *vals) -> None:
        value = self._extract_first_value(vals)
        if value is None:
            self.get_logger().warning("%s handler received non-numeric payload: %s", addr, vals)
            return
        clamped = self._clamp(value, ALTITUDE_INPUT_RANGE)
        for name in self.crafts:
            self._apply_altitude(name, clamped, source="global")

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

    # ------------------------------------------------------------------
    # CrazySwarm service helpers
    # ------------------------------------------------------------------
    def _apply_lateral(self, craft: str, normalized: float, source: str) -> None:
        state = self.crafts[craft]
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


def main() -> None:
    rclpy.init()
    node = SwarmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        node.stop_osc_server()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
