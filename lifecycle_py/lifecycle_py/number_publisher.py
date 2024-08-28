#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from example_interfaces.msg import Int64


class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.get_logger().info("In Constructor")
        self.number_ = 1
        self.publish_frequency_ = 1.0
        self.number_publisher_ = None
        self.number_timer_ = None
        
    # Create ROS2 communications, connect to HW
    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("In on_configure callback")
        # Lifecycle publishers are not active unless the node is in the active state
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "number", 10)
        self.number_timer_ = self.create_timer(
            1.0/self.publish_frequency_, self.publish_number)
        return TransitionCallbackReturn.SUCCESS

    # Activate/Enable HW. It is important to call the super().on_activate func
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("In on_activate callback")
        self.number_timer_.reset()
        return super().on_activate(previous_state)

    # Deactivate/Disable HW. It is important to call the super().on_deactivate func
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("In on_deactivate callback")
        self.number_timer_.cancel()
        return super().on_deactivate(previous_state)

    # Destroy ROS2 communications, disconnect from HW
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("In on_cleanup callback")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS

    # Destroy ROS2 communications, disconnect from HW (just for safety)
    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("In on_shutdown callback")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return super().on_shutdown(previous_state)

    # Process erros, deactivate + cleanup
    def on_error(self, previous_state: LifecycleState):
        self.get_logger().info("In on_error callback")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)

        # Do some checks. If ok, then return SUCCESS. If not, failure
        return TransitionCallbackReturn.FAILURE

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.number_+=1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()