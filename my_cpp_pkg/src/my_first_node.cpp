#include "rclcpp/rclcpp.hpp"


class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("cpp_test"), counter_(0)
    {
        // Print log
        RCLCPP_INFO(this->get_logger(),"Hello CPP Node");

        timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                                         std::bind(&MyNode::timerCallback, this));
        
    }

private:

    void timerCallback()
    {
        counter_++;
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};



int main(int argc, char **argv)
{
    // Initialize ROS2 communication
    rclcpp::init(argc, argv);

    // Create a pointer for a node obj
    auto node = std::make_shared<MyNode>();

    // Pause the program and keep the node alive
    rclcpp::spin(node);

    // Node will be shutdown
    rclcpp::shutdown();

    return 0;
}