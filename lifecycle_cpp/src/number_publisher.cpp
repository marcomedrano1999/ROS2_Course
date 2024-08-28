#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


class NumberPublisherNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    NumberPublisherNode() : LifecycleNode("number_publisher")
    {
        number_ = 1;
        publish_frequency_ = 1.0;
        RCLCPP_INFO(this->get_logger(), "In constructor");
    }

    // Create ROS2 communications, connect to HW
    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "In on_configure callback");
        number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number",10);
        number_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/publish_frequency_)),
                                                std::bind(&NumberPublisherNode::publishNumber, this));
        number_timer_->cancel();
        return LifecycleCallbackReturn::SUCCESS;
    }

    //  Destroy ROS2 communications, disconnect from HW
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "In on_cleanup callback");
        number_publisher_.reset();
        number_timer_.reset();
        return LifecycleCallbackReturn::SUCCESS;
    }

    // Activate/Enable HW. It is important to call the LifecycleNode::on_activate func
    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "In on_activate callback");
        number_timer_->reset();
        rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    // Deactivate/Disable HW. It is important to call the LifecycleNode::on_deactivate func
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "In on_deactivate callback");
        number_timer_->cancel();
        rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    //  Destroy ROS2 communications, disconnect from HW (just for safety)
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "In on_shutdown callback");
        number_publisher_.reset();
        number_timer_.reset();
        return LifecycleCallbackReturn::SUCCESS;
    }

    // Process erros, deactivate + cleanup
    LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "In on_error callback");
        number_publisher_.reset();
        number_timer_.reset();

        return LifecycleCallbackReturn::FAILURE;
    }

private:

    void publishNumber()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        number_publisher_->publish(msg);
        number_++;
    }
    
    int number_;
    double publish_frequency_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
    rclcpp::TimerBase::SharedPtr number_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}