#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_robot.hpp"
#include <mutex>

using MoveRobot = my_robot_interfaces::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ServerGoalHandle<MoveRobot>;

class MoveRobotServerNode : public rclcpp::Node 
{
public:
    MoveRobotServerNode() : Node("move_robot_server") 
    {
        this->robot_position = 50;
        this->cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        move_robot_server_ = rclcpp_action::create_server<MoveRobot>(
            this,
            "move_robot",
            std::bind(&MoveRobotServerNode::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveRobotServerNode::cancel_callback, this, std::placeholders::_1),
            std::bind(&MoveRobotServerNode::handle_accept_callback, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            cb_group_
        );
        RCLCPP_INFO(this->get_logger(), "The action server has been started");
    }

private:

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveRobot::Goal> goal
    )
    {
        (void) uuid;
        
        // Validate the goal request
        if(goal->velocity <= 0 || goal->position < 0 || goal->position > 100)
        {
            RCLCPP_INFO(this->get_logger(), "Rejecting the goal due to invalid velocity/position");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Policy: preempt current goal is a new one arrives
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(this->goal_handle_)
            {
                if(this->goal_handle_->is_active())
                {
                    RCLCPP_INFO(this->get_logger(), "Aborting current goal and executing new goal");
                    preempted_goal_id_ = this->goal_handle_->get_goal_id();
                }
            }
        }


        RCLCPP_INFO(this->get_logger(), "Accepting the goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<MoveRobotGoalHandle> goal_handle
    )
    {
        RCLCPP_INFO(this->get_logger(), "Received a cancel request");
        (void) goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accept_callback(const std::shared_ptr<MoveRobotGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing the goal");
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<MoveRobotGoalHandle> goal_handle)
    {

        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }

        // Get request from handle
        int goal_position = goal_handle->get_goal()->position;
        int velocity = goal_handle->get_goal()->velocity;

        // Execute the action
        auto result = std::make_shared<MoveRobot::Result>();
        auto feedback = std::make_shared<MoveRobot::Feedback>();
        rclcpp::Rate loop_rate(1.0);

        int distance = goal_position-this->robot_position;
        int steps = abs(distance/velocity);
        steps = ( (distance%velocity) == 0) ? steps :(steps+1);

        for(int i=0; i < steps; i++)
        {

            {
                std::lock_guard<std::mutex> lock(mutex_);
                // Check for goal preemption
                if(goal_handle->get_goal_id() == preempted_goal_id_)
                {
                    result->position = this->robot_position;
                    result->message = "Goal preempted by a new goal";
                    goal_handle->abort(result);
                    return;
                }
            }

            // Check for goal cancelation
            if(goal_handle->is_canceling()){
                result->position = this->robot_position;
                result->message = "Goal cancelled";
                goal_handle->canceled(result);
                return;
            }

            if(abs(this->robot_position - goal_position) < velocity)
            {
                this->robot_position = goal_position;
            }
            else{
                this->robot_position += (distance > 0) ? velocity : -velocity;
            }

            // Send feedback
            RCLCPP_INFO(this->get_logger(), "%d", this->robot_position);
            feedback->current_position = this->robot_position;
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }

        // Set final state and return result
        result->position = this->robot_position;
        result->message = "Goal reached";
        goal_handle->succeed(result);
    }

    rclcpp_action::Server<MoveRobot>::SharedPtr move_robot_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::shared_ptr<MoveRobotGoalHandle> goal_handle_;
    int robot_position;
    std::mutex mutex_;
    rclcpp_action::GoalUUID preempted_goal_id_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    //rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}