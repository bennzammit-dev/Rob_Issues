import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

# In your launch description add:
SetEnvironmentVariable(name='HOME', value=os.environ['HOME']),

def generate_launch_description():
    pkg_share = get_package_share_directory('sortabot')
    world_file = os.path.join(pkg_share, 'worlds', 'sortabot.world')
    
    # Simple launch - just Gazebo
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
            env={'LIBGL_ALWAYS_SOFTWARE': '1'}
        ),
    ])