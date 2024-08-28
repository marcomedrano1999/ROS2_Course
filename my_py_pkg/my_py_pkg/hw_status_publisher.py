#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import HardwareStatus

class HardwareStatusPublisherNode(Node): 
    def __init__(self):
        super().__init__("hardware_status_publisher")

        # Create publisher obj
        self.publisher_ = self.create_publisher(HardwareStatus, "hardware_status", 10) # type, topic name, queue size

        # add timer
        self.timer_ = self.create_timer(0.5, self.publish_hardware_status)

        self.get_logger().info("Hardware status Publisher has been started")

    def publish_hardware_status(self):
        # Create msg obj
        msg = HardwareStatus()
        # Add custom data
        msg.temperature = 25
        msg.are_motors_ready = True
        msg.debug_message = "Nothing special"

        # Output the data
        self.publisher_.publish(msg)
        self.get_logger().info("Message published!")


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisherNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()