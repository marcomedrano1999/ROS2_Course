#include "rclcpp/rclcpp.hpp"
#include <string>

#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

#define MAX_X           11.0
#define MAX_Y           11.0
#define MAX_THETA       (3.1416*2)

class TurtleSpawnerNode : public rclcpp::Node 
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner") 
    {
        this->declare_parameter("spawn_frequency", 1.0);
        this->declare_parameter("turtle_name_prefix","turtle_");
        this->spawn_frequency = this->get_parameter("spawn_frequency").as_double();
        this->turtle_name_prefix = this->get_parameter("turtle_name_prefix").as_string();

        this->turtle_counter = 0;

        publisher_ = this->create_publisher<my_robot_interfaces::msg::TurtleArray>("alive_turtles",10);

        server_ = this->create_service<my_robot_interfaces::srv::CatchTurtle>(
            "catch_turtle",
            std::bind(&TurtleSpawnerNode::callbackCatchTurtle, this,std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(std::chrono::milliseconds((long)(1000.0/this->spawn_frequency)), std::bind(&TurtleSpawnerNode::createNewTurtle, this));
    }

    void publishAliveTurtles()
    {
        // Only publish if there is at least one turtle in the array
        if(turtle_array.size() == 0) return;

        auto msg = my_robot_interfaces::msg::TurtleArray();
        msg.turtles = turtle_array;
        publisher_->publish(msg);
    }

    void callKillService(std::string name)
    {
        // Create the client
        auto client = this->create_client<turtlesim::srv::Kill>("kill");

        // Wait for the server to be up
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        // Create the request
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;

        // Send the request
        auto future = client->async_send_request(request);

        try{
            // Wait for the response
            auto response = future.get();

            // Erase element from vector
            if(turtle_array.size() <= 1)
            {
                turtle_array.clear();
            }
            else
            {
                for(size_t i = 0; i<turtle_array.size(); i++)
                {
                    RCLCPP_INFO(this->get_logger(), "transversing element");        
                    if(request->name == turtle_array[i].name)
                    {
                        RCLCPP_INFO(this->get_logger(), "Before erasing");
                        auto it = turtle_array.begin() + i;
                        turtle_array.erase(it);
                        RCLCPP_INFO(this->get_logger(), "After erasing");
                    }
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Killed turtle %s",request->name.c_str());

            // Publish turtle array
            publishAliveTurtles();
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
                
    }

    void callbackCatchTurtle(const my_robot_interfaces::srv::CatchTurtle::Request::SharedPtr request,
                            const my_robot_interfaces::srv::CatchTurtle::Response::SharedPtr response)
    {
        response->success = false;

        for(auto it=turtle_array.begin(); it != turtle_array.end(); it++)
        {
            if(request->name == it->name)
            {
                response->success = true;

                // Call kill service
                threads_.push_back(std::thread(std::bind(&TurtleSpawnerNode::callKillService, this, request->name)));

                break;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Request turtle %s kill resulted in %s", 
                    request->name.c_str(),
                    (response->success ? "Success" : "Failure"));
    }



    void callSpawnService()
    {
        // Create the client
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");

        // Wait for the server to be up
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        // Create the request
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = ((float)(rand()) / (float)(RAND_MAX)) * MAX_X;
        request->y = ((float)(rand()) / (float)(RAND_MAX)) * MAX_Y;
        request->theta = ((float)(rand()) / (float)(RAND_MAX)) * MAX_THETA;
        request->name = this->turtle_name_prefix+std::to_string(turtle_counter);

        turtle_counter++;

        // Send the request
        auto future = client->async_send_request(request);

        try{
            // Wait for the response
            auto response = future.get();

            RCLCPP_INFO(this->get_logger(), "Created turtle %s in (%f, %f, %f)",
                        response->name.c_str(), request->x, request->y, request->theta);

            // Save new turtle in array
            my_robot_interfaces::msg::Turtle newTurtle;
            newTurtle.x = request->x;
            newTurtle.y = request->y;
            newTurtle.theta = request->theta;
            newTurtle.name = response->name;
            turtle_array.push_back(newTurtle);

            // Publish turtle array
            publishAliveTurtles();
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    void createNewTurtle()
    {
        threads_.push_back(std::thread(std::bind(&TurtleSpawnerNode::callSpawnService, this)));
    }

private:
    double spawn_frequency;
    std::string turtle_name_prefix;
    int turtle_counter;
    rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr publisher_;
    rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr server_;
    std::vector<my_robot_interfaces::msg::Turtle> turtle_array;
    std::vector<std::thread> threads_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();  
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}