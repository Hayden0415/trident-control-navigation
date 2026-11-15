"""Interactive kinematic environment: never sends a goal by default."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='forest', choices=['forest','wall']),
        DeclareLaunchArgument('auto_goal', default_value='false'),
        Node(package='nav_simulation', executable='course_demo', output='screen',
             parameters=[{'scene': LaunchConfiguration('scene'),
                          'auto_goal': LaunchConfiguration('auto_goal')}]),
    ])
