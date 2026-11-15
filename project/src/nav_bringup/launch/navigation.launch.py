from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        *[IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(pkg),'launch',name])),
                                  launch_arguments={'use_sim_time':LaunchConfiguration('use_sim_time')}.items())
          for pkg,name in [('nav_planning','planning.launch.py'),('nav_legacy_control','control.launch.py')]],
    ])
