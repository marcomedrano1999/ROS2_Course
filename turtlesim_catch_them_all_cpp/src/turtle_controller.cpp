#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

class TurtleControllerNode : public rclcpp::Node 
{
public:
    TurtleControllerNode() : Node("turtle_controller") 
    {

        this->declare_parameter("catch_closest_turtle_first", false);
        this->declare_parameter("vel_gain", 1.0);
        this->gain = this->get_parameter("vel_gain").as_double();
        this->catch_closest_turtle_first = this->get_parameter("catch_closest_turtle_first").as_bool();

        this->publish_frequency_ = 1.0;

        this->turtleToHunt_.x = 0.0;
        this->turtleToHunt_.y = 0.0;
        this->turtleToHunt_.theta = 0;
        this->isTurtleHunting = false;

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel",10);

        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,
                      std::bind(&TurtleControllerNode::callback_pose, this, std::placeholders::_1));

        aliveTurtles_subscriber_ = this->create_subscription<my_robot_interfaces::msg::TurtleArray>("alive_turtles", 10,
                      std::bind(&TurtleControllerNode::callback_aliveTurtles, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Turtle controller has been started.");
    }

private:

    void callCatchTurtleService(std::string name)
    {
        // Create the client
        auto client = this->create_client<my_robot_interfaces::srv::CatchTurtle>("catch_turtle");

        // Wait for the server to be up
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        // Create the request
        auto request = std::make_shared<my_robot_interfaces::srv::CatchTurtle::Request>();
        request->name = name;

        // Send the request
        auto future = client->async_send_request(request);

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

    void callback_aliveTurtles(const my_robot_interfaces::msg::TurtleArray::SharedPtr msg){

        RCLCPP_INFO(this->get_logger(), "Turtle controller received turtle array");

        // Check if the turtle is already hunting 
        if(isTurtleHunting) return;
        
        // Check if there is at least one turtle in the array
        if(msg->turtles.size() == 0) return;

        if(msg->turtles.size() == 1 || !catch_closest_turtle_first)
        {
            turtleToHunt_ = msg->turtles[0];
        }
        else
        {
            // Very high double value
            double distance = 10000000.0;
            for(auto turtle : msg->turtles)
            {
                double x_diff = currentPose_.x - turtle.x;
                double y_diff = currentPose_.y - turtle.y;
                double turtle_distance = sqrt(x_diff*x_diff + y_diff*y_diff);
                RCLCPP_INFO(this->get_logger(), "Turtle %s is %lf from the main turtle", turtle.name.c_str(), turtle_distance);

                if(turtle_distance < distance)
                {
                    turtleToHunt_ = turtle;
                    distance = turtle_distance;
                }
            }
        }

        isTurtleHunting = true;
    }


    void callback_pose(const turtlesim::msg::Pose::SharedPtr current_pose)
    {
        this->currentPose_ = *current_pose.get();

        this->publish_cmd_vel();
    }

    void publish_cmd_vel()
    {
        // Only move if there is at least one turtle to hunt
        if(!isTurtleHunting) return;

        auto msg = geometry_msgs::msg::Twist();

        double x_diff = turtleToHunt_.x - currentPose_.x;
        double y_diff = turtleToHunt_.y - currentPose_.y;


        double distance = sqrt(pow(x_diff,2) + pow(y_diff,2));
        if(distance > 0.5)
        {
            msg.linear.x = distance*this->gain;

            double steering_angle = std::atan2(y_diff, x_diff);
            double angle_diff = steering_angle - currentPose_.theta;
            if(angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }
            else if(angle_diff < -M_PI)
            {
                angle_diff += 2*M_PI;
            }
            msg.angular.z = 6*angle_diff;
        }
        else
        {
            msg.linear.x = 0;
            msg.angular.z = 0;

            // Call catch_turtle srv to erase the reached turtle
            threads_.push_back(std::thread(std::bind(&TurtleControllerNode::callCatchTurtleService, this, turtleToHunt_.name)));

            isTurtleHunting = false;
        }
        
        publisher_->publish(msg);
    }


    int number_;
    float gain;
    double publish_frequency_;
    bool isTurtleHunting;
    bool catch_closest_turtle_first;
    turtlesim::msg::Pose currentPose_;
    my_robot_interfaces::msg::Turtle turtleToHunt_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr aliveTurtles_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();   
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}