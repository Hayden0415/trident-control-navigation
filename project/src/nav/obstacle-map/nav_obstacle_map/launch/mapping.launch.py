"""Interactive student mapper, optionally fused with the half-pillar prior."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = FindPackageShare('nav_obstacle_map')
    prior = LaunchConfiguration('use_global_prior')
    clock = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('inflation_radius', default_value='0.4'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_global_prior', default_value='true'),
        DeclareLaunchArgument('prior_map', default_value=PathJoinSubstitution([share,'resource','forest-half-prior.json'])),
        Node(package='nav_obstacle_map', executable='student_mapper', output='screen',
             parameters=[PathJoinSubstitution([share,'config','mapping.yaml']),clock,
                         {'inflation_radius': ParameterValue(LaunchConfiguration('inflation_radius'),value_type=float)}],
             remappings=[('/nav/map', PythonExpression(["'/nav/map/local' if '",prior,"' == 'true' else '/nav/map'"]))]),
        Node(package='nav_obstacle_map', executable='global_prior', output='screen',
             parameters=[{'prior_map':LaunchConfiguration('prior_map')},clock], condition=IfCondition(prior)),
    ])
