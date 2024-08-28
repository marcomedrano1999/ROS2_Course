#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class CounUntilServerNode(Node): 
    def __init__(self):
        super().__init__("count_until_server")
        self.goal_handler_ :ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.goal_queue = []
        self.count_until_server_ = ActionServer(self, CountUntil, "count_until",
                                                goal_callback=self.goal_callback,
                                                handle_accepted_callback=self.handle_accepted_callback,
                                                cancel_callback=self.cancel_callback,
                                                execute_callback=self.execute_callback,
                                                callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Action server has been started")

    def goal_callback(self, goal_request : CountUntil.Goal):
        self.get_logger().info("Received a goal")

        # Policy: refuse new goal is current goal is active
        # with self.goal_lock_:
        #     if self.goal_handler_ is not None and self.goal_handler_.is_active:
        #         self.get_logger().info("A goal is ready active. Rejecting new goal")
        #         return GoalResponse.REJECT

        # Validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().info("Rejecting the goal")
            return GoalResponse.REJECT
        
        # Policy: preempt existing goal when receiving new goal
        # with self.goal_lock_:
        #     if self.goal_handler_ is not None and self.goal_handler_.is_active:
        #         self.get_logger().info("Abort current goal and accept new goal")
        #         self.goal_handler_.abort()

        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handler : ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handler_ is not None:
                self.goal_queue.append(goal_handler)
            else:
                goal_handler.execute()

    def execute_callback(self, goal_handler: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handler_ = goal_handler

        # Get request from the goal
        target_number = goal_handler.request.target_number
        period = goal_handler.request.period

        # Execute the action
        self.get_logger().info("Executing the goal")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter  = 0

        for i in range(target_number):
            # Chec for goal abortion
            if not goal_handler.is_active:
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result
            # Check for goal cancellation
            if goal_handler.is_cancel_requested:
                self.get_logger().info("Canceling the goal")
                goal_handler.canceled()
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result
            counter+=1
            feedback._current_number = counter
            goal_handler.publish_feedback(feedback)
            self.get_logger().info(str(counter))
            time.sleep(period)
    
        # Once we're done, set goal final state
        goal_handler.succeed()
        #goal_handler.abort()

        # And send the result
        
        result.reached_number = counter
        self.process_next_goal_in_queue()
        return result
    
    def cancel_callback(self, goal_handle : ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT
    
    def process_next_goal_in_queue(self):
        with self.goal_lock_:
            if len(self.goal_queue) > 0:
                self.goal_queue.pop(0).execute()
            else:
                self.goal_handler_ = None

def main(args=None):
    rclpy.init(args=args)
    node = CounUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__=="__main__":
    main()