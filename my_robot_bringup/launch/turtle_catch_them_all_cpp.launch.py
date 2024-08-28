from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtle_env_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtle_env_node"
    )

    turtle_spawner_node = Node(
        package="turtlesim_catch_them_all_cpp",
        executable="turtle_spawner",
        name="turtle_spawner",
        parameters=[
            {"spawn_frequency":0.8}
        ]
    )

    turtle_controller_node = Node(
        package="turtlesim_catch_them_all_cpp",
        executable="turtle_controller",
        name="turtle_controller",
        parameters=[
            {"catch_closest_turtle_first":True}
        ]
    )

    ld.add_action(turtle_env_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtle_controller_node)
    
    return ld