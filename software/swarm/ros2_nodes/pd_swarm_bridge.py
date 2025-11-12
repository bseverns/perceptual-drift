"""ROS 2 bridge that turns Perceptual Drift OSC mirrors into CrazySwarm2 calls."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32

from crazyflie_interfaces.srv import GoTo, Land, Stop, Takeoff


@dataclass
class DroneState:
    """Track a drone's target and simulated pose for telemetry."""

    name: str
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    target: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    yaw: float = 0.0
    target_yaw: float = 0.0

    def step_toward_target(self, alpha: float) -> None:
        for idx in range(3):
            delta = self.target[idx] - self.position[idx]
            self.position[idx] += delta * alpha
        yaw_delta = self.target_yaw - self.yaw
        self.yaw += yaw_delta * alpha

    def to_pose(self, node: Node) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.position
        msg.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.pose.orientation.z = math.sin(self.yaw / 2.0)
        return msg


class PdSwarmBridge(Node):
    """Subscribe to `/pd/*` topics and command a swarm in sim or hardware."""

    def __init__(self) -> None:
        super().__init__("pd_swarm_bridge")

        self.declare_parameter("drones", ["cf1", "cf2", "cf3"])
        self.declare_parameter("lateral_axis", "y")
        self.declare_parameter("lateral_range", [-1.0, 1.0])
        self.declare_parameter("lateral_scale_m", 0.8)
        self.declare_parameter("altitude_range", [0.0, 1.0])
        self.declare_parameter("altitude_floor_m", 0.3)
        self.declare_parameter("altitude_scale_m", 1.0)
        self.declare_parameter("yaw_scale_deg", 90.0)
        self.declare_parameter("group_mask", 0)
        self.declare_parameter("relative_moves", False)
        self.declare_parameter("go_to_duration_s", 0.75)
        self.declare_parameter("takeoff_duration_s", 1.5)
        self.declare_parameter("land_duration_s", 2.0)
        self.declare_parameter("publish_rate_hz", 20.0)

        names = self.get_parameter("drones").get_parameter_value().string_array_value
        self.drones: Dict[str, DroneState] = {name: DroneState(name=name) for name in names}
        for index, drone in enumerate(self.drones.values()):
            drone.target[0] = (index - (len(self.drones) - 1) / 2.0) * 1.5
            drone.position[0] = drone.target[0]

        self.go_to_clients = {name: self._create_client(GoTo, f"/{name}/go_to") for name in names}
        self.takeoff_clients = {name: self._create_client(Takeoff, f"/{name}/takeoff") for name in names}
        self.land_clients = {name: self._create_client(Land, f"/{name}/land") for name in names}
        self.stop_clients = {name: self._create_client(Stop, f"/{name}/stop") for name in names}

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10,
        )

        self.subscription_alt = self.create_subscription(Float32, "/pd/alt", self.on_alt, qos)
        self.subscription_lat = self.create_subscription(Float32, "/pd/lat", self.on_lat, qos)
        self.subscription_yaw = self.create_subscription(Float32, "/pd/yaw", self.on_yaw, qos)
        self.subscription_consent = self.create_subscription(Bool, "/pd/consent", self.on_consent, qos)

        self.publishers = {
            name: self.create_publisher(PoseStamped, f"/pd/sim/{name}/pose", qos)
            for name in names
        }

        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self._publish_states)

        self._lateral_axis = self.get_parameter("lateral_axis").get_parameter_value().string_value.lower()
        self._lateral_range = tuple(self.get_parameter("lateral_range").get_parameter_value().double_array_value)
        self._lateral_scale = self.get_parameter("lateral_scale_m").get_parameter_value().double_value
        self._altitude_range = tuple(self.get_parameter("altitude_range").get_parameter_value().double_array_value)
        self._altitude_floor = self.get_parameter("altitude_floor_m").get_parameter_value().double_value
        self._altitude_scale = self.get_parameter("altitude_scale_m").get_parameter_value().double_value
        self._yaw_scale = math.radians(self.get_parameter("yaw_scale_deg").get_parameter_value().double_value)
        self._group_mask = self.get_parameter("group_mask").get_parameter_value().integer_value
        self._relative = self.get_parameter("relative_moves").get_parameter_value().bool_value
        self._go_to_duration = self.get_parameter("go_to_duration_s").get_parameter_value().double_value
        self._takeoff_duration = self.get_parameter("takeoff_duration_s").get_parameter_value().double_value
        self._land_duration = self.get_parameter("land_duration_s").get_parameter_value().double_value

        self._consent_enabled = True

        self.get_logger().info("PD swarm bridge ready for %s", ", ".join(self.drones.keys()))

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def on_alt(self, msg: Float32) -> None:
        value = self._clamp(msg.data, self._altitude_range)
        target_alt = self._altitude_floor + value * self._altitude_scale
        self.get_logger().debug("/pd/alt -> %.2fm", target_alt)
        for name, drone in self.drones.items():
            drone.target[2] = target_alt
            self._call_takeoff(name, target_alt)

    def on_lat(self, msg: Float32) -> None:
        value = self._clamp(msg.data, self._lateral_range)
        span = (len(self.drones) - 1) / 2.0
        for idx, (name, drone) in enumerate(self.drones.items()):
            formation_offset = (idx - span) * 0.8
            axis_index = 1 if self._lateral_axis == "y" else 0
            drone.target[axis_index] = value * self._lateral_scale + formation_offset
            self._call_go_to(name, drone)

    def on_yaw(self, msg: Float32) -> None:
        value = self._clamp(msg.data, (-1.0, 1.0))
        for drone in self.drones.values():
            drone.target_yaw = value * self._yaw_scale
        self.get_logger().debug("/pd/yaw -> %.1f deg", math.degrees(self.drones[next(iter(self.drones))].target_yaw))

    def on_consent(self, msg: Bool) -> None:
        self._consent_enabled = bool(msg.data)
        if not self._consent_enabled:
            self.get_logger().warning("Consent revoked — landing all drones and locking inputs")
            for name, drone in self.drones.items():
                drone.target[2] = 0.0
                self._call_land(name)
                self._call_stop(name)
        else:
            self.get_logger().info("Consent restored — controls re-enabled")

    # ------------------------------------------------------------------
    # Service helpers
    # ------------------------------------------------------------------
    def _create_client(self, srv_type, name: str):
        client = self.create_client(srv_type, name)
        return client

    def _wait_for_service(self, client, label: str) -> bool:
        if client.wait_for_service(timeout_sec=0.0):
            return True
        self.get_logger().debug("%s not ready", label)
        return False

    def _call_takeoff(self, name: str, height: float) -> None:
        if not self._consent_enabled:
            return
        client = self.takeoff_clients[name]
        if not self._wait_for_service(client, f"{name}/takeoff"):
            return
        req = Takeoff.Request()
        if hasattr(req, "height"):
            req.height = height
        self._apply_duration(req, self._takeoff_duration)
        if hasattr(req, "group_mask"):
            req.group_mask = self._group_mask
        client.call_async(req)

    def _call_go_to(self, name: str, drone: DroneState) -> None:
        if not self._consent_enabled:
            return
        client = self.go_to_clients[name]
        if not self._wait_for_service(client, f"{name}/go_to"):
            return
        req = GoTo.Request()
        goal = None
        if hasattr(req, "goal"):
            goal = req.goal
        elif hasattr(req, "target"):
            goal = req.target
        if goal is not None:
            goal.x = drone.target[0]
            goal.y = drone.target[1]
            goal.z = drone.target[2]
        if hasattr(req, "yaw"):
            req.yaw = drone.target_yaw
        self._apply_duration(req, self._go_to_duration)
        if hasattr(req, "relative"):
            req.relative = self._relative
        if hasattr(req, "group_mask"):
            req.group_mask = self._group_mask
        client.call_async(req)

    def _call_land(self, name: str) -> None:
        client = self.land_clients[name]
        if not self._wait_for_service(client, f"{name}/land"):
            return
        req = Land.Request()
        if hasattr(req, "height"):
            req.height = 0.0
        self._apply_duration(req, self._land_duration)
        client.call_async(req)

    def _call_stop(self, name: str) -> None:
        client = self.stop_clients[name]
        if not self._wait_for_service(client, f"{name}/stop"):
            return
        req = Stop.Request()
        client.call_async(req)

    # ------------------------------------------------------------------
    # Telemetry loop
    # ------------------------------------------------------------------
    def _publish_states(self) -> None:
        for name, drone in self.drones.items():
            drone.step_toward_target(alpha=0.15)
            msg = drone.to_pose(self)
            self.publishers[name].publish(msg)

    @staticmethod
    def _clamp(value: float, bounds: Iterable[float]) -> float:
        lower, upper = bounds
        return max(lower, min(upper, value))

    def _apply_duration(self, req, seconds: float) -> None:
        if not hasattr(req, "duration"):
            return
        duration = getattr(req, "duration")
        ros_duration = Duration(seconds=float(seconds))
        if hasattr(duration, "sec") and hasattr(duration, "nanosec"):
            duration.sec = int(ros_duration.nanoseconds // 1_000_000_000)
            duration.nanosec = int(ros_duration.nanoseconds % 1_000_000_000)
        else:
            setattr(req, "duration", float(seconds))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PdSwarmBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
