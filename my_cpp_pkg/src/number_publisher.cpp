#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPublisherNode : public rclcpp::Node 
{
public:
    NumberPublisherNode() : Node("number_publisher") 
    {
        this->declare_parameter("number_to_publish", 2);
        this->declare_parameter("publish_frequency", 1.0);
        this->number_ = this->get_parameter("number_to_publish").as_int();
        this->publish_frequency_ = this->get_parameter("publish_frequency").as_double();
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number",10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/this->publish_frequency_)), std::bind(&NumberPublisherNode::publishNumber, this));
        RCLCPP_INFO(this->get_logger(), "Number Publisher has been started.");
    }

    void publishNumber()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = this->number_;
        publisher_->publish(msg);
    }

private:
    int number_;
    double publish_frequency_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();   
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}