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

import rclpy
from rclpy.node import Node
from pythonosc import dispatcher, osc_server
from crazyflie_interfaces.srv import Takeoff, GoTo


class SwarmNode(Node):
    """ROS 2 node that listens for OSC and relays to a single Crazyflie.

    Extend this class with additional service clients (e.g. Land, Stop) and
    subscriptions if you want feedback from motion capture or onboard sensors.
    """

    def __init__(self):
        super().__init__("swarm_node")

        # CrazySwarm2 exposes one service per drone.  ``/cf1/go_to`` moves CF #1.
        # See ``ros2 service list`` for the services available in your workspace.
        self.go_to = self.create_client(GoTo, "/cf1/go_to")
        self.takeoff = self.create_client(Takeoff, "/cf1/takeoff")

        # Placeholder: bolt OSC handlers onto this dispatcher.
        self.dispatcher = dispatcher.Dispatcher()
        self.dispatcher.map("/pd/lat", self.on_lat)
        self.dispatcher.map("/pd/alt", self.on_alt)

        # Fire up a background OSC server.  Threading server keeps ROS spinning.
        self.server = osc_server.ThreadingOSCUDPServer(("0.0.0.0", 9010), self.dispatcher)
        self.get_logger().info("OSC listening on %s:%d", *self.server.server_address)

    # ------------------------------------------------------------------
    # OSC handlers — currently just log the incoming values.  Replace the
    # TODOs with calls to ``self.go_to.call_async`` to actually move drones.
    # ------------------------------------------------------------------
    def on_lat(self, addr, *vals):
        self.get_logger().info("Received lateral input: %.3f", float(vals[0]))
        # TODO: translate into GoTo requests with x/y offsets.

    def on_alt(self, addr, *vals):
        self.get_logger().info("Received altitude input: %.3f", float(vals[0]))
        # TODO: call self.takeoff / modify z targets.


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
