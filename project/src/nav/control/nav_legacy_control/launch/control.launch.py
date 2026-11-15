from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='nav_legacy_control', executable='controller_node', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('nav_legacy_control'),'config','control.yaml']),
                         {'use_sim_time':LaunchConfiguration('use_sim_time')}]),
    ])
