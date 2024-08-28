#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
from my_robot_interfaces.srv import SetLed

class BatteryNode(Node): 
    def __init__(self):
        super().__init__("battery") 

        # add timer
        self.timer_ = self.create_timer(4, self.set_led_on)

        

    def set_led_on(self):
        # Execute the client
        self.call_set_led_server(3,True)
        self.timer_.cancel()
        self.timer_ = self.create_timer(6, self.set_led_off)

    def set_led_off(self):
        # Execute the client
        self.call_set_led_server(3,False)
        self.timer_.cancel()
        self.timer_ = self.create_timer(4, self.set_led_on)

    def call_set_led_server(self, led_number, state):
        client = self.create_client(SetLed, "set_led")
    
        # Wait for the server
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server set_led...")

        # Create a request object
        request = SetLed.Request()
        request.led_number = led_number
        request.state = state

        # Use async call as recommended
        future = client.call_async(request)

        # Add done callback to execute when future is complete
        # Use partial to add more arguments
        future.add_done_callback(partial(self.callback_call_set_led, led_number=led_number, state=state))

    def callback_call_set_led(self, future, led_number, state):
        try: 
            response = future.result()
            self.get_logger().info(f"Led number {str(led_number)} requested to {'ON' if state else 'OFF'} resulted in {'Success' if response.success else 'Failure'}")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))



def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()