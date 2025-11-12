"""Push simulated swarm poses into Ignition Gazebo."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class PdSimPoseDriver(Node):
    """Apply the pd_swarm_bridge output poses to Gazebo models."""

    def __init__(self) -> None:
        super().__init__("pd_sim_pose_driver")

        self.declare_parameter("world", "pd_swarm_world")
        self.declare_parameter("drones", ["cf1", "cf2", "cf3"])

        world = self.get_parameter("world").get_parameter_value().string_value
        names = self.get_parameter("drones").get_parameter_value().string_array_value
        self.pose_client = self.create_client(SetEntityPose, f"/world/{world}/set_pose")

        for name in names:
            topic = f"/pd/sim/{name}/pose"
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, drone=name: self._handle_pose(drone, msg),
                10,
            )

    def _handle_pose(self, drone: str, msg: PoseStamped) -> None:
        if not self.pose_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().debug("Gazebo pose service unavailable; skipping %s", drone)
            return
        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = drone
        req.entity.type = Entity.MODEL
        req.pose = msg.pose
        self.pose_client.call_async(req)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PdSimPoseDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
