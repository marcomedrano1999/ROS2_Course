cmake_minimum_required(VERSION 3.8)
project(turtlesim_catch_them_all_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

# Create the component
add_library(turtle_advanced_controller SHARED src/turtle_advanced_controller.cpp)
ament_target_dependencies(turtle_advanced_controller rclcpp turtlesim rclcpp_action 
             my_robot_interfaces geometry_msgs rclcpp_lifecycle rclcpp_components)
rclcpp_components_register_nodes(turtle_advanced_controller "TurtleAdvancedController::TurtleAdvancedController")

add_library(turtlebot_controller SHARED src/turtlebot_controller.cpp)
ament_target_dependencies(turtlebot_controller rclcpp turtlesim rclcpp_action 
             my_robot_interfaces geometry_msgs rclcpp_lifecycle rclcpp_components)
rclcpp_components_register_nodes(turtlebot_controller "TurtleBotController::TurtleBotController")

add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller rclcpp geometry_msgs turtlesim my_robot_interfaces)

add_executable(turtle_spawner src/turtle_spawner.cpp)
ament_target_dependencies(turtle_spawner rclcpp turtlesim my_robot_interfaces)

#add_executable(turtle_advanced_controller src/turtle_advanced_controller.cpp)
#ament_target_dependencies(turtle_advanced_controller rclcpp turtlesim rclcpp_action 
#              my_robot_interfaces geometry_msgs rclcpp_lifecycle)

# This install is only for components
install(TARGETS
  turtle_advanced_controller
  turtlebot_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(TARGETS
  turtle_controller
  turtle_spawner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
