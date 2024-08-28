#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import MoveRobot
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

class MoveRobotServeNode(LifecycleNode): 
    def __init__(self):
        super().__init__("move_robot_server")
        self.declare_parameter("action_server_name", rclpy.Parameter.Type.STRING)
        self.action_server_name_ = self.get_parameter("action_server_name").value
        self.goal_handler_ :ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.robot_position_ = 55
        self.is_robot_accepting_goals = False

        self.get_logger().info("In constructor")
        self.get_logger().info("Robot position: "+str(self.robot_position_))

    def on_configure(self, previous_state : LifecycleState):
        self.get_logger().info("In on_configure callback")
        self.move_robot_server_ = ActionServer(
            self,
            MoveRobot,
            self.action_server_name_,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Action server has been started")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, previous_state : LifecycleState):
        self.get_logger().info("In on_cleanup callback")
        self.move_robot_server_.destroy()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state : LifecycleState):
        self.get_logger().info("In on_activation callback")
        self.is_robot_accepting_goals = True
        return super().on_activate(previous_state)

    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("In on_deactivate callback")
        self.is_robot_accepting_goals = False
        with self.goal_lock_:
            if self.goal_handler_ is not None and self.goal_handler_.is_active:
                self.goal_handler_.abort()
        return super().on_deactivate(previous_state)

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("In on_shutdown callback")
        self.move_robot_server_.destroy()
        return super().on_shutdown(state)

    def cancel_callback(self, goal_handle : ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT

    def goal_callback(self, goal_request : MoveRobot.Goal):
        self.get_logger().info("Received a goal")

        # Reject any new goals if node is not in active state
        if(not self.is_robot_accepting_goals):
            self.get_logger().warn("Rejecting the goal due to node being not active")
            return GoalResponse.REJECT

        # Validate the goal request
        if goal_request.velocity <= 0 or goal_request.position not in range(0,100):
            self.get_logger().warn("Rejecting the goal due to invalid position/velocity")
            return GoalResponse.REJECT
        
        # Policy: preempt existing goal when receiving new goal
        with self.goal_lock_:
            if self.goal_handler_ is not None and self.goal_handler_.is_active:
                self.get_logger().info("Abort current goal and accept new goal")
                self.goal_handler_.abort()
        
        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle : ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handler_ = goal_handle

        # Get request from goal
        goal_position = goal_handle.request.position
        velocity = goal_handle.request.velocity
        
        # Execute the action
        self.get_logger().info("Executing the action")
        feedback = MoveRobot.Feedback()
        result = MoveRobot.Result()
        distance = goal_position - self.robot_position_
        steps = int(abs(distance / velocity))
        steps += 0 if((distance%velocity)==0) else 1
        
        for i in range(steps):

            # Check for abortion
            if not goal_handle.is_active:
                result.position = self.robot_position_
                result.message = "Preempted by another goal, or node is deactivated"
                return result
            
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling the goal")
                goal_handle.canceled()
                result.position = self.robot_position_
                result.message = "Goal was cancelled"
                return result

            if(abs(self.robot_position_ - goal_position) < velocity):
                self.robot_position_ = goal_position
            else:
                self.robot_position_ += velocity if distance > 0 else -velocity
            
            # Send feedback
            feedback.current_position = self.robot_position_
            self.get_logger().info("Robot position: "+str(self.robot_position_))
            goal_handle.publish_feedback(feedback)
            
            time.sleep(1.0)
        
        goal_handle.succeed()
        
        # Send the result
        result.position = self.robot_position_
        result.message = "Goal reached"
        return result



def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotServeNode() 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__=="__main__":
    main()