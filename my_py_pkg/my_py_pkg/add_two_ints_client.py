#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node): 
    def __init__(self):
        super().__init__("add_two_ints_client") 
        # Execute the client
        self.call_add_two_ints_server(6,7)

    def call_add_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")
    
        # Wait for the server
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server AddTwoInts...")

        # Create a request object
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Use async call as recommended
        future = client.call_async(request)

        # Add done callback to execute when future is complete
        # Use partial to add more arguments
        future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))

    def callback_call_add_two_ints(self, future, a, b):
        try: 
            response = future.result()
            self.get_logger().info(str(a) + " + " + str(b) + " = " + 
                                str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))



def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()