cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(turtlesim_catch_them_all_cpp REQUIRED)
find_package(my_robot_description REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(lifecycle_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
