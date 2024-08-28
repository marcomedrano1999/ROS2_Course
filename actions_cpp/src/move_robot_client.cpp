#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_robot.hpp"
#include "my_robot_interfaces/msg/cancel_robot_move.hpp"

using MoveRobot = my_robot_interfaces::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ClientGoalHandle<MoveRobot>;


class MoveRobotClientNode : public rclcpp::Node 
{
public:
    MoveRobotClientNode() : Node("move_robot_client") 
    {
        this->cancel_subscriber_ = this->create_subscription<my_robot_interfaces::msg::CancelRobotMove>(
            "cancel_move",
            10,
            std::bind(&MoveRobotClientNode::callback_cancel_move_robot, this, std::placeholders::_1)
        );

        this->move_robot_client_ = rclcpp_action::create_client<MoveRobot>(
            this,
            "move_robot"
        );
    }

    void send_goal(int position, int velocity)
    {
        // Wait for the action server
        move_robot_client_->wait_for_action_server();

        // Create the goal
        auto goal = MoveRobot::Goal();
        goal.position = position;
        goal.velocity = velocity;

        // Add callbacks
        auto options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();
        options.feedback_callback = 
            std::bind(&MoveRobotClientNode::goal_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        options.goal_response_callback = 
            std::bind(&MoveRobotClientNode::goal_response_callback, this, std::placeholders::_1);
        options.result_callback = 
            std::bind(&MoveRobotClientNode::goal_result_callback, this, std::placeholders::_1);

        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending a goal");
        move_robot_client_->async_send_goal(goal, options);
    }

private:

    // Callback for cancel subscription
    void callback_cancel_move_robot(const my_robot_interfaces::msg::CancelRobotMove::SharedPtr msg)
    {
        (void)msg;
        if(this->goal_handle_)
        {
            RCLCPP_INFO(this->get_logger(), "Sending cancel request");
            move_robot_client_->async_cancel_goal(this->goal_handle_);
            this->goal_handle_.reset();
        }
    }

    // Callback to know if the goal request was accepted or rejected
    void goal_response_callback(const MoveRobotGoalHandle::SharedPtr &goal_handle)
    {
        if(!goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Goal got rejected");
        }
        else{
            this->goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal got accepted");
        }
    }

    // Callback to receive the result once the goal was done
    void goal_result_callback(const MoveRobotGoalHandle::WrappedResult &result)
    {
        auto status = result.code;

        if(status==rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Succeded");
        }
        else if(status==rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        }
        else if(status==rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_WARN(this->get_logger(), "Canceled");
        }

        int position = result.result->position;
        std::string message = result.result->message;
        RCLCPP_INFO(this->get_logger(), "Result: position: %d, message: %s", position, message.c_str());
    }

    void goal_feedback_callback(const MoveRobotGoalHandle::SharedPtr &goal_handle,
        const std::shared_ptr<const MoveRobot::Feedback> &feedback)
    {
        (void)goal_handle;
        int current_position = feedback->current_position;
        RCLCPP_INFO(this->get_logger(), "Got feedback: %d", current_position);
    }

    rclcpp_action::Client<MoveRobot>::SharedPtr move_robot_client_;
    MoveRobotGoalHandle::SharedPtr goal_handle_;
    rclcpp::Subscription<my_robot_interfaces::msg::CancelRobotMove>::SharedPtr cancel_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotClientNode>(); 
    node->send_goal(20, 3);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}