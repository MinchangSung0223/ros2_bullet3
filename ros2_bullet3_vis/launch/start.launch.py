from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='ros2_bullet3_rt',
            executable='ros2_bullet3_rt',
            name='ros2_bullet3_rt'
        ),
        Node(
            package='ros2_bullet3_control',
            executable='ros2_bullet3_control',
            name='ros2_bullet3_control'
        ),Node(
            package='ros2_bullet3_vis',
            executable='ros2_bullet3_vis',
            name='ros2_bullet3_vis'
        ),
        ExecuteProcess(
        cmd=['rqt'],
        )
    ])
