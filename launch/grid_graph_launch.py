from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchContext


def generate_launch_description():
    node = Node(
        package="graph_env",
        executable="graph_env",
        output="screen",
        parameters=[
            {
                "graph_type": "grid",
                "num_nodes": 300,
                "num_branches": 8,
                "connectivity_percent": 0.4,
                "graph_seed": 0
            }
        ]
    )
    return LaunchDescription([node])
