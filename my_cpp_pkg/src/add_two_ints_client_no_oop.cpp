#include "rclcpp/rclcpp.hpp"

#include "example_interfaces/srv/add_two_ints.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("Add_two_ints_client_no_oop");   // MODIFY NAME
    
    // Create the client
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    // Wait for the server to be up
    while(!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(node->get_logger(), "Waiting for the server to be up...");
    }

    // Create the request
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();

    // Fill the request
    request->a = 3;
    request->b = 8;

    // Send the request
    auto future = client->async_send_request(request);

    // Wait for the server's reponse
    if(rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "%ld + %ld = %ld", request->a, request->b, future.get()->sum);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Error while calling serviice");
    }

    rclcpp::shutdown();
    return 0;
}