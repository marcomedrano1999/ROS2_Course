#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import SetLed
from my_robot_interfaces.msg import LedState

class LedPanelNode(Node): 
    def __init__(self):
        super().__init__("led_panel") 

        self.declare_parameter("led_states", [0, 0, 0])

        self.led_state = self.get_parameter("led_states").value

        ## Create a server
        self.server_ = self.create_service(
            SetLed,"set_led", self.callback_set_led) # Srv_type, name_of_srv, callback
        self.get_logger().info("Set led server has been started")
        
        # Create publisher obj
        self.publisher_ = self.create_publisher(LedState, "led_panel_state", 10) # type, topic name, queue size

        # add timer
        self.timer_ = self.create_timer(0.5, self.publish_led_state)

        self.get_logger().info("Number Publisher has been started")

    def publish_led_state(self):
        # Create msg obj
        msg = LedState()
        # Add custom data
        for i in range(3):
            msg.led_state[i] = self.led_state[i]
        # Output the data
        self.publisher_.publish(msg)
        self.get_logger().info("Message published!")

    def callback_set_led(self, request, response):

        if(request.led_number > 0 and request.led_number <=3):
            self.led_state[request.led_number - 1] = 1 if (request.state) else 0
            response.success = True
        else:
            response.success = False
        
        self.get_logger().info(f"Request led number {str(request.led_number)} to {'ON' if request.state else 'OFF'} is {'Success' if response.success else 'Failure'}")
        return response     ## Always return the response

def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()