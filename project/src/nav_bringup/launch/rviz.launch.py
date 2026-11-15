"""Native RViz2 debugger; SetGoal publishes directly to the assignment goal topic."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d',PathJoinSubstitution([FindPackageShare('nav_bringup'),'config','course.rviz'])],
             parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time')}]),
    ])
