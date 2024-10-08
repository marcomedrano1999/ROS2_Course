cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/LedState.msg"
  "msg/Turtle.msg"
  "msg/TurtleArray.msg"
  "msg/CancelRobotMove.msg"
  "srv/ComputeRectangleArea.srv"
  "srv/SetLed.srv"
  "srv/CatchTurtle.srv"
  "action/CountUntil.action"
  "action/MoveRobot.action"
  "action/MoveTurtle.action"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
