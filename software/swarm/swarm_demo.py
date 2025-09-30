#!/usr/bin/env python3
# Example CrazySwarm2 script driven by OSC
import rclpy
from rclpy.node import Node
from pythonosc import dispatcher, osc_server
from crazyflie_interfaces.srv import Takeoff, GoTo

class SwarmNode(Node):
    def __init__(self):
        super().__init__('swarm_node')
        # This would wire into CrazySwarm2 service calls
        self.cli = self.create_client(GoTo, '/cf1/go_to')
        # Placeholder for OSC integration...

def main():
    rclpy.init()
    node = SwarmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
