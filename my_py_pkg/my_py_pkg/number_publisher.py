#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Import data type
from example_interfaces.msg import Int64

class NumberPublisherNode(Node): 
    def __init__(self):
        super().__init__("number_publisher")

        self.declare_parameter("number_to_publish", 9)
        self.declare_parameter("publish_frequency", 1.0)
        
        self.number_ = self.get_parameter("number_to_publish").value
        self.publish_frequency_ = self.get_parameter("publish_frequency").value

        # Create publisher obj
        self.publisher_ = self.create_publisher(Int64, "number", 10) # type, topic name, queue size

        # add timer
        self.timer_ = self.create_timer(1.0/self.publish_frequency_, self.publish_number)

        self.get_logger().info("Number Publisher has been started")

    def publish_number(self):
        # Create msg obj
        msg = Int64()
        # Add custom data
        msg.data = self.number_
        # Output the data
        self.publisher_.publish(msg)
        self.get_logger().info("Message published!")


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()