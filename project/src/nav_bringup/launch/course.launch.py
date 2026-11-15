"""Final automatic wall-fixture acceptance; manual development uses package launches."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    diagnostic=LaunchConfiguration('diagnostic_grid');prior=LaunchConfiguration('use_global_prior')
    clock={'use_sim_time':LaunchConfiguration('use_sim_time')}
    map_topic=PythonExpression(["'/nav/map/local' if '",prior,"' == 'true' else '/nav/map'"])
    return LaunchDescription([
        DeclareLaunchArgument('diagnostic_grid', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('demo', default_value='true'),
        DeclareLaunchArgument('use_global_prior', default_value='false'),
        DeclareLaunchArgument('prior_map', default_value=PathJoinSubstitution([FindPackageShare('nav_obstacle_map'),'resource','forest-half-prior.json'])),
        Node(package='nav_planning',executable='planner_node',parameters=[clock],output='screen'),
        Node(package='nav_legacy_control',executable='controller_node',parameters=[clock],output='screen'),
        Node(package='nav_obstacle_map',executable='student_mapper',
             parameters=[PathJoinSubstitution([FindPackageShare('nav_obstacle_map'),'config','mapping.yaml']),clock],
             remappings=[('/nav/map',map_topic)],condition=UnlessCondition(diagnostic),output='screen'),
        Node(package='nav_obstacle_map',executable='global_prior',parameters=[{'prior_map':LaunchConfiguration('prior_map')},clock],
             condition=IfCondition(prior),output='screen'),
        Node(package='nav_simulation',executable='course_demo',parameters=[{'diagnostic_grid':diagnostic},clock],
             condition=IfCondition(LaunchConfiguration('demo')),output='screen'),
    ])
