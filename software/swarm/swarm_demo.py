#!/usr/bin/env python3
"""Sketch of an OSC-driven CrazySwarm2 node.

CrazySwarm2 (https://crazyswarm.readthedocs.io/) exposes ROS 2 services for
controlling Bitcraze Crazyflies.  This file outlines how Perceptual Drift could
hand off the gesture data to a fleet instead of a single Betaflight whoop.

This is not a complete swarm choreographer — it is a heavily commented starting
point.  You will need a ROS 2 Foxy/Humble workspace with the crazyflie_ros2 stack
installed.  Recommended reading:

* CrazySwarm2 tutorial — https://imrclab.github.io/crazyswarm/usage/
* python-osc quickstart — https://pypi.org/project/python-osc/
* ROS 2 services overview — https://docs.ros.org/en/humble/Concepts/Services.html
"""

import math
import threading
from typing import Iterable, Optional

import rclpy
from rclpy.node import Node
from pythonosc import dispatcher, osc_server
from crazyflie_interfaces.srv import Takeoff, GoTo

# ---------------------------------------------------------------------------
# Tweakables — change these to match your swarm layout or taste.
# ---------------------------------------------------------------------------

# OSC server binding — Perceptual Drift publishes to this IP/port combo.
OSC_BIND_ADDRESS = "0.0.0.0"
OSC_BIND_PORT = 9010

# ROS 2 service names exposed by crazyflie_ros2.
GO_TO_SERVICE = "/cf1/go_to"
TAKEOFF_SERVICE = "/cf1/takeoff"

# Lateral motion scaling: incoming [-1, 1] values become X/Y offsets in meters.
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



class SwarmNode(Node):
    """ROS 2 node that listens for OSC and relays to a single Crazyflie.

    Extend this class with additional service clients (e.g. Land, Stop) and
    subscriptions if you want feedback from motion capture or onboard sensors.
    """

    def __init__(self):
        super().__init__("swarm_node")

        # CrazySwarm2 exposes one service per drone.  ``/cf1/go_to`` moves CF #1.
        # See ``ros2 service list`` for the services available in your workspace.
        self.go_to = self.create_client(GoTo, GO_TO_SERVICE)
        self.takeoff = self.create_client(Takeoff, TAKEOFF_SERVICE)

        self._current_altitude = ALTITUDE_FLOOR_M
        self._current_lateral = LATERAL_HOME_M

        # Placeholder: bolt OSC handlers onto this dispatcher.
        self.dispatcher = dispatcher.Dispatcher()
        self.dispatcher.map("/pd/lat", self.on_lat)
        self.dispatcher.map("/pd/alt", self.on_alt)

        # Fire up a background OSC server.  Threading server keeps ROS spinning.
        self.server = osc_server.ThreadingOSCUDPServer((OSC_BIND_ADDRESS, OSC_BIND_PORT), self.dispatcher)
        self._osc_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self._osc_thread.start()
        self.get_logger().info("OSC listening on %s:%d", *self.server.server_address)

    # ------------------------------------------------------------------
    # OSC handlers — translate crowd gestures into CrazySwarm2 service calls.
    # ------------------------------------------------------------------
    def on_lat(self, addr, *vals):
        value = self._extract_first_value(vals)
        if value is None:
            self.get_logger().warning("/pd/lat handler received non-numeric payload: %s", vals)
            return

        clamped = self._clamp(value, LATERAL_INPUT_RANGE)
        if not math.isclose(clamped, value, rel_tol=1e-6, abs_tol=1e-6):
            self.get_logger().warn(
                "Lateral input %.3f clipped to %.3f (range %.2f..%.2f)",
                value,
                clamped,
                *LATERAL_INPUT_RANGE,
            )

        self._current_lateral = LATERAL_HOME_M + clamped * LATERAL_SCALE_M

        if not self._wait_for_service(self.go_to, "GoTo"):
            return

        req = GoTo.Request()
        if LATERAL_AXIS.lower() == "x":
            self._set_position(req, axis="x", value=self._current_lateral)
            self._set_position(req, axis="y", value=LATERAL_HOME_M)
        else:
            self._set_position(req, axis="y", value=self._current_lateral)
            self._set_position(req, axis="x", value=LATERAL_HOME_M)
        # Preserve altitude when issuing lateral moves so we do not dip.
        self._set_position(req, axis="z", value=self._current_altitude)
        self._assign_if_present(req, "duration", GO_TO_DURATION_S)
        self._assign_if_present(req, "yaw", 0.0)
        self._assign_if_present(req, "relative", RELATIVE_MOVES)
        self._assign_if_present(req, "group_mask", GROUP_MASK)

        self.get_logger().info(
            "Commanding %s-axis slide to %.2fm (alt=%.2fm)",
            LATERAL_AXIS,
            self._current_lateral,
            self._current_altitude,
        )
        self.go_to.call_async(req)

    def on_alt(self, addr, *vals):
        value = self._extract_first_value(vals)
        if value is None:
            self.get_logger().warning("/pd/alt handler received non-numeric payload: %s", vals)
            return

        clamped = self._clamp(value, ALTITUDE_INPUT_RANGE)
        if not math.isclose(clamped, value, rel_tol=1e-6, abs_tol=1e-6):
            self.get_logger().warn(
                "Altitude input %.3f clipped to %.3f (range %.2f..%.2f)",
                value,
                clamped,
                *ALTITUDE_INPUT_RANGE,
            )

        height = ALTITUDE_FLOOR_M + clamped * ALTITUDE_SCALE_M
        self._current_altitude = height

        if not self._wait_for_service(self.takeoff, "Takeoff"):
            return

        req = Takeoff.Request()
        applied = self._assign_if_present(req, "height", height)
        if not applied:
            # Fallback for alternative field naming (e.g. target.z).
            self._set_position(req, axis="z", value=height)
        self._assign_if_present(req, "duration", TAKEOFF_DURATION_S)
        self._assign_if_present(req, "group_mask", GROUP_MASK)

        self.get_logger().info("Commanding takeoff to %.2fm", height)
        self.takeoff.call_async(req)

    # ------------------------------------------------------------------
    # Helper utilities
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

    def _set_position(self, req, *, axis: str, value: float) -> None:
        candidates = ("goal", "position", "target")
        axis = axis.lower()
        for field in candidates:
            if not hasattr(req, field):
                continue
            target = getattr(req, field)
            if hasattr(target, axis):
                setattr(target, axis, value)
                return
        self.get_logger().debug("Request missing %s axis field; left unchanged", axis)

    @staticmethod
    def _assign_if_present(msg, field: str, value) -> bool:
        if hasattr(msg, field):
            setattr(msg, field, value)
            return True
        return False

    def destroy_node(self):  # type: ignore[override]
        if hasattr(self, "server"):
            try:
                self.server.shutdown()
                self.server.server_close()
            except Exception:  # pragma: no cover - best effort cleanup
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = SwarmNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
