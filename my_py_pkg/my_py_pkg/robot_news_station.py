#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Import data type
from example_interfaces.msg import String

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station") # MODIFY NAME

        self.declare_parameter("robot_name", "C3PO")

        # Add robot name
        self.robot_name_ = self.get_parameter("robot_name").value

        # Create publisher obj
        self.publisher_ = self.create_publisher(String, "robot_news", 10) # type, topic name, queue size

        # add timer
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Robot News Station has been started")

    def publish_news(self):
        # Create msg obj
        msg = String()
        # Add custom data
        msg.data = f"Hi, this is {self.robot_name_}"
        # Output the data
        self.publisher_.publish(msg)

    
def main(args=None):
    # Init ROS2 communications
    rclpy.init(args=args)
    # Create node
    node = RobotNewsStationNode() # MODIFY NAME
    # Keep node alive
    rclpy.spin(node)
    # Free mem
    rclpy.shutdown()


if __name__=="__main__":
    main()