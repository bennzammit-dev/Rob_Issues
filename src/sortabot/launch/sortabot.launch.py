from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory("sortabot"),
        "worlds",
        "sortabot_base.world"
    )

    return LaunchDescription([

        ExecuteProcess(
            cmd=[
                "gz", "sim",
                "--verbose",
                "-r",
                world_path
            ],
            output="screen"
        ),

        Node(
            package="sortabot",
            executable="world_manager",
            output="screen"
        ),

        Node(
            package="sortabot",
            executable="robot_controller",
            output="screen"
        ),

        Node(
            package="sortabot",
            executable="childabot",
            output="screen"
        ),
    ])