#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Import data type
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class NumberCounterNode(Node): 
    def __init__(self):
        super().__init__("number_counter") 

        self.counter_ = 0

        self.subscriber_ = self.create_subscription(Int64, "number",self.callback_number, 10)
        
        # Create publisher obj
        self.publisher_ = self.create_publisher(Int64, "number_count", 10) # type, topic name, queue size

        self.server_ = self.create_service(
            SetBool,"reset_counter", self.callback_reset_counter) # Srv_type, name_of_srv, callback
        self.get_logger().info("Reset counter server has been started")

    def callback_reset_counter(self, request, response):
        if(request.data):
            self.counter_ = 0
            self.get_logger().info("Reset counter")
            response.success = True
        return response     ## Always return the response

    def callback_number(self, msg):
        
        self.counter_+=1

        # Create msg obj
        msg = Int64()
        # Add custom data
        msg.data = self.counter_
        # Output the data
        self.publisher_.publish(msg)
        self.get_logger().info(str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()