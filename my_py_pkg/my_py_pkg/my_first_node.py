#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class MyNode(Node):

    def __init__(self):
        super().__init__("py_test")

        # add counter
        self.counter = 0

        # Use logger to print text
        self.get_logger().info("Hello ROS2!!!")

        # create a timer to execute periodically
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info("Hello " + str(self.counter))


def main(args=None):

    # Start the ros communication
    rclpy.init(args=args)

    # Create a node
    node = MyNode()

    # will paused the program and continue to be alive
    rclpy.spin(node)

    # Last sentence to suspend communication
    rclpy.shutdown()


if __name__=="__main__":
    main()