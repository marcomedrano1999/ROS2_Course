#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class NumberCounterNode : public rclcpp::Node 
{
public:
    NumberCounterNode() : Node("number_counter") 
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10,
                      std::bind(&NumberCounterNode::callback_number, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count",10);
        RCLCPP_INFO(this->get_logger(), "Number Counter has been started.");

        server_ = this->create_service<example_interfaces::srv::SetBool>(
            "reset_counter",
            std::bind(&NumberCounterNode::callbackResetNumberCount, this,std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Service server has been started");
    }

private:
    void callback_number(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        (void)msg;
        auto publish_msg = example_interfaces::msg::Int64();
        publish_msg.data = ++this->counter_;
        publisher_->publish(publish_msg);

        RCLCPP_INFO(this->get_logger(), "%ld", publish_msg.data);
    }

    void callbackResetNumberCount(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                            const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if(request->data)
        {
            this->counter_ = 0;
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Resetting the counter to zero");
        }
    }

    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;    

    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    int counter_=0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();   // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}