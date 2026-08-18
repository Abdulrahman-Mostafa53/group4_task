from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    movex_action_server_node = Node(
        executable="move_x_action_server",
        package="maze_solver",
        name="movex_action_server",
    )
    move_yaw_action_server_node = Node(
        executable="move_yaw_action_server",
        package="maze_solver",
        name="yaw_action_server",
    )
    action_client_node = Node(
        executable="action", package="maze_solver", name="action_client"
    )
    return LaunchDescription(
        [movex_action_server_node, move_yaw_action_server_node, action_client_node]
    )
