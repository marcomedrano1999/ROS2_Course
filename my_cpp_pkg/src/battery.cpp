#include "rclcpp/rclcpp.hpp"

#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode : public rclcpp::Node // MODIFY NAME
{
public:
    BatteryNode() : Node("battery") // MODIFY NAME
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(6000), std::bind(&BatteryNode::turnOnLed, this));
    }

    void turnOnLed()
    {
        // Create a thread to manage asyncronous request to the server
        thread1_ = std::thread(std::bind(&BatteryNode::callSetLedService,this,3,true));
        thread1_.detach();
        timer_->cancel();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(4000), std::bind(&BatteryNode::turnOffLed, this));
    }

    void turnOffLed()
    {
        thread2_ = std::thread(std::bind(&BatteryNode::callSetLedService,this,3,false));
        thread2_.detach();
        timer_->cancel();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(6000), std::bind(&BatteryNode::turnOnLed, this));
    }



    void callSetLedService(int led_number, bool state)
    {
        // Create the client
        auto client = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");

        // Wait for the server to be up
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        // Create the request
        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request->led_number = led_number;
        request->state = state;

        // Send the request
        auto future = client->async_send_request(request);

        try{
            // Wait for the response
            auto response = future.get();

            RCLCPP_INFO(this->get_logger(), "Change led %d to state %s with a result of %s",
                        led_number,(state? "ON" : "OFF"),(response->success ? "Success" : "Failure"));
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }



private:
    std::thread thread1_, thread2_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();   // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}