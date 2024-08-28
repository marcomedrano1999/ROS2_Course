#include "rclcpp/rclcpp.hpp"
#include <string>
#include <mutex>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "my_robot_interfaces/action/move_turtle.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

#define MAX_X           11.0
#define MAX_Y           11.0
#define MAX_THETA       (3.1416*2)

#define ROBOT_INITIAL_X         5.5
#define ROBOT_INITIAL_Y         5.5
#define ROBOT_INITIAL_THETA     0.0

using MoveTurtle = my_robot_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;
using LifecycleCallbackReturn = 
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace TurtleAdvancedController{

class TurtleAdvancedController : public rclcpp_lifecycle::LifecycleNode
{
public:
    TurtleAdvancedController(const rclcpp::NodeOptions &options) : LifecycleNode("turtle_advanced_controller", options)
    {
        this->declare_parameter("turtle_name","turtle_1");
        this->declare_parameter("kill_initial_node", true);
        
        // Turtle name
        this->turtle_name_ = this->get_parameter("turtle_name").as_string();

        // Kill the default turtle
        this->kill_initial_turtle = this->get_parameter("kill_initial_node").as_bool();

        this->is_server_up = false;
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    }

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "In on_configure callback");
        // Create the kill turtleclient
        kill_turtle_client_ = this->create_client<turtlesim::srv::Kill>("kill",rmw_qos_profile_services_default,cb_group_);
        // Create the spawn turtle client
        spawn_turtle_client_ = this->create_client<turtlesim::srv::Spawn>("spawn",rmw_qos_profile_services_default,cb_group_);
        // Create publisher for velocity commands
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(turtle_name_+"/cmd_vel",10);
        // Create publisher for position updates
        auto subscriber_options = rclcpp::SubscriptionOptions();
        subscriber_options.callback_group = cb_group_;
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(turtle_name_+"/pose", 10,
                      std::bind(&TurtleAdvancedController::callback_pose, this, std::placeholders::_1),subscriber_options);
        // Action server
        move_turtle_server_ = rclcpp_action::create_server<MoveTurtle>(
            this,
            turtle_name_+"/move_turtle",
            std::bind(&TurtleAdvancedController::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TurtleAdvancedController::cancel_callback, this, std::placeholders::_1),
            std::bind(&TurtleAdvancedController::handle_accept_callback, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            cb_group_
        );
        RCLCPP_INFO(this->get_logger(), "The action server has been started");

        // Kill the default turtle
        if(kill_initial_turtle)
        {
            callKillService("turtle1");
            kill_initial_turtle = false;
        }
        // Create a turtle
        callSpawnService(turtle_name_);

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "In on_cleanup callback");
        // Kill this turtle
        callKillService(this->turtle_name_);
        kill_turtle_client_.reset();
        spawn_turtle_client_.reset();
        publisher_.reset();
        pose_subscriber_.reset();
        move_turtle_server_.reset();
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "In on_active callback");
        this->is_server_up = true;
        rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "In on_deactive callback");
        this->is_server_up = false;

        // Deactivate current goal
        {
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);
            if(this->goal_handle_)
            {
                if(this->goal_handle_->is_active())
                {
                    RCLCPP_INFO(this->get_logger(), "Aborting current goal due to server deactivation");
                    this->is_goal_aborted = true;
                }
            }
        }
        rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "In on_shutdown callback");
        this->is_server_up = false;
        // Kill this turtle
        callKillService(this->turtle_name_);
        kill_turtle_client_.reset();
        spawn_turtle_client_.reset();
        publisher_.reset();
        pose_subscriber_.reset();
        move_turtle_server_.reset();
        return LifecycleCallbackReturn::SUCCESS;
    }

private:

    void callKillService(std::string name)
    {
        // Wait for the server to be up
        while(!kill_turtle_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        // Create the request
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;

        // Send the request
        auto future = kill_turtle_client_->async_send_request(request);

        try{
            // Wait for the response
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Killed turtle %s",request->name.c_str());
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }   
    }

    void callSpawnService(std::string name)
    {
        // Wait for the server to be up
        while(!spawn_turtle_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        // Create the request
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = ROBOT_INITIAL_X; 
        request->y = ROBOT_INITIAL_Y; 
        request->theta = ROBOT_INITIAL_THETA; 
        request->name = name;

        // Send the request
        auto future = spawn_turtle_client_->async_send_request(request);

        try{
            // Wait for the response
            auto response = future.get();

            RCLCPP_INFO(this->get_logger(), "Created turtle %s in (%f, %f, %f)",
                        response->name.c_str(), request->x, request->y, request->theta);
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveTurtle::Goal> goal
    )
    {
        (void)uuid;

        // Accept goal only in active state
        if(!this->is_server_up)
        {
            RCLCPP_INFO(this->get_logger(), "Rejecting due to server down");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Goal policy: rejecting new goals
        {
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);
            if(this->goal_handle_)
            {
                if(this->goal_handle_->is_active())
                {
                    RCLCPP_INFO(this->get_logger(), "Rejecting new goal due to already active goal");
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
        }
        
        // Validate the request
        if(goal->duration <= 0.0)
        {
            RCLCPP_INFO(this->get_logger(), "Rejecting the goal due to invalid goal duration");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Accepting the goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<MoveTurtleGoalHandle> goal_handle
    )
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received a cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accept_callback(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing the goal");
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle)
    {
        // Save goal handle
        {
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);
            this->goal_handle_ = goal_handle;
        }

        // Get request from handle
        double goal_linear_vel_x = goal_handle->get_goal()->linear_vel_x;
        double angular_vel_z = goal_handle->get_goal()->angular_vel_z;
        double duration = goal_handle->get_goal()->duration;

        // Execute the action
        auto result = std::make_shared<MoveTurtle::Result>();
        auto feedback = std::make_shared<MoveTurtle::Feedback>();
        rclcpp::Rate loop_rate(10.0);
        double seconds = 0.0;
        
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = goal_linear_vel_x;
        msg.angular.z = angular_vel_z;

        while(seconds < duration)
        {
            // Check for goal abortion
            {
                std::lock_guard<std::mutex> lock(goal_handle_mutex_);
                if(this->is_goal_aborted)
                {
                    this->is_goal_aborted=false;
                    result->position = this->robot_position_;
                    result->message = "Goal cancelled";
                    goal_handle->abort(result);
                    return;
                }
            }
            
            // Check for goal cancellation
            if(goal_handle->is_canceling())
            {
                result->position = this->robot_position_;
                result->message = "Goal cancelled";
                goal_handle->canceled(result);
                return;
            }

            // Send the cmd_vel
            publisher_->publish(msg);
            seconds+=0.1;

            // Send feedback
            {
                std::lock_guard<std::mutex> lock(position_mutex_);
                RCLCPP_INFO(this->get_logger(), "Robot position: x=%f, y=%f, thetha=%f",
                            this->robot_position_[0], this->robot_position_[1], this->robot_position_[2]);
                feedback->current_position = this->robot_position_;
            }
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }

        // Set final state and return result
        result->message = "Goal reached";
        {
            std::lock_guard<std::mutex> lock(position_mutex_);
            result->position = this->robot_position_;
            RCLCPP_INFO(this->get_logger(), "Final robot position: x=%f, y=%f, thetha=%f",
                            this->robot_position_[0], this->robot_position_[1], this->robot_position_[2]);
        }
        goal_handle->succeed(result);
    }

    void callback_pose(const turtlesim::msg::Pose::SharedPtr current_pose)
    {
        std::lock_guard<std::mutex> lock(position_mutex_);
        // Save current robot position for feedback
        this->robot_position_[0] = (*current_pose.get()).x;
        this->robot_position_[1] = (*current_pose.get()).y;
        this->robot_position_[2] = (*current_pose.get()).theta;
    }

    bool is_server_up;
    bool is_goal_aborted;
    bool kill_initial_turtle;
    std::mutex position_mutex_;
    std::mutex goal_handle_mutex_;
    std::string turtle_name_;
    std::vector<double> robot_position_{ROBOT_INITIAL_X,ROBOT_INITIAL_Y,ROBOT_INITIAL_THETA};
    std::shared_ptr<MoveTurtleGoalHandle> goal_handle_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_turtle_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_turtle_client_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp_action::Server<MoveTurtle>::SharedPtr move_turtle_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
};

} // namespace TurtleAdvancedController

/*int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleAdvancedControllerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}*/

// Add these lines to register the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(TurtleAdvancedController::TurtleAdvancedController)