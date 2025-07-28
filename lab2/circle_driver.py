#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import # <message_type> # TODO: what type of message is used to drive the robot?
import signal, time, sys


class CircleDriver(Node):
    def __init__(self):
        super().__init__('circle_driver')
        self.pub = self.create_publisher(<message_type>, '<topic>', 10) # TODO: use the message type from the import; what topic should it publish to?

        self.cmd = <message_type>() # TODO
        self.cmd.linear.x  = 0.1
        self.cmd.angular.z = 0.1
        radius = self.cmd.linear.x / abs(self.cmd.angular.z)

        self.timer = self.create_timer(0.1, lambda: self.pub.publish(self.cmd))
        self.get_logger().info(f"driving in a circle: v={self.cmd.linear.x:.2f}m/s, z={self.cmd.angular.z:.2f}rad/s: r={radius:.2f}m")

    def stop_robot(self):
        self.pub.publish(<message_type>()) # TODO - new messages will have all 0 values - stopping the robot
        time.sleep(0.1)
        self.get_logger().info("robot stopped")


def main(args=None):
    rclpy.init(args=args)
    node = CircleDriver()

    def custom_sigint(signum, frame):
        node.get_logger().info("Ctrl-C: stopping robot")
        node.stop_robot()
        rclpy.shutdown()
        signal.signal(signal.SIGINT, signal.SIG_IGN)

    signal.signal(signal.SIGINT, custom_sigint)

    rclpy.spin(node)
    node.destroy_node()
    sys.exit(0)


if __name__ == "__main__":
    main()
