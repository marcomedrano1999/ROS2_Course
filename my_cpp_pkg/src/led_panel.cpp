#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_state.hpp"


class LedPanelNode : public rclcpp::Node 
{
public:
    LedPanelNode() : Node("led_panel") 
    {
        this->declare_parameter("led_states", std::vector<int>({0, 0, 0}));
        this->led_state = this->get_parameter("led_states").as_integer_array();

        server_ = this->create_service<my_robot_interfaces::srv::SetLed>(
            "set_led",
            std::bind(&LedPanelNode::callbackSetLed, this,std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Led panel server has been started");

        publisher_ = this->create_publisher<my_robot_interfaces::msg::LedState>("led_panel_state",10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&LedPanelNode::publishLedState, this));
        RCLCPP_INFO(this->get_logger(), "Led state Publisher has been started.");
    }


    void callbackSetLed(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                            const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        if(request->led_number > 0 && request->led_number <= 3)
        {
            this->led_state[request->led_number - 1] = (request->state ? 1 : 0) ;
            response->success = true;
        }
        else
        {
            response->success = false;
        }

        RCLCPP_INFO(this->get_logger(), "Request led number %ld to %s resulted in %s", 
                    request->led_number,
                    (request->state ? "ON" : "OFF"),
                    (response->success ? "Success" : "Failure"));
    }

    void publishLedState()
    {
        auto msg = my_robot_interfaces::msg::LedState();
        msg.led_state[0] = this->led_state[0];
        msg.led_state[1] = this->led_state[1];
        msg.led_state[2] = this->led_state[2];
        publisher_->publish(msg);
    }

private:

    std::vector<int64_t> led_state;
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;

    rclcpp::Publisher<my_robot_interfaces::msg::LedState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}