"""Gazebo Fortress physics, native GUI and real simulated LiDAR; no automatic goal."""
import os
import re
import tempfile
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav_simulation.world import prepare


def start(context):
    val=lambda name:LaunchConfiguration(name).perform(context)
    own=Path(get_package_share_directory('nav_simulation'))
    urdf,world,models=prepare(tempfile.mkdtemp(prefix='course-gazebo-'),x=float(val('x')),y=float(val('y')),
                             yaw=float(val('yaw')),z=.4,seed=20261027,rate=float(val('real_time_factor')),
                             wheel_control_mode='velocity_motor',map_id='ego_pillar_forest_slim')
    gui_config=Path(world).parent/'gui.config'
    stock=Path('/usr/share/ignition/ignition-gazebo6/gui/gui.config').read_text()
    stock=re.sub(r'<camera_pose>.*?</camera_pose>','<camera_pose>34 22 26 0 0.9 -2.5</camera_pose>',stock)
    stock=stock.replace('show_again="true"','show_again="false"')
    gui_config.write_text(stock)
    command=['ign','gazebo','-v','3','-r','--gui-config',str(gui_config),world]
    if val('gui').lower()!='true':command=['ign','gazebo','-s','-r','--headless-rendering',world]
    prefix=own.parents[1]
    return [
        SetEnvironmentVariable('IGN_PARTITION',val('partition')),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',models+':'+os.environ.get('IGN_GAZEBO_RESOURCE_PATH','')),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',str(prefix/'lib')+':'+os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH','')),
        ExecuteProcess(cmd=command,output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(own/'config/bridge.yaml'),'use_sim_time':True}],output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':urdf,'frame_prefix':'sentry/','use_sim_time':True}],remappings=[('joint_states','/sentry/joint_states')],output='screen'),
        Node(package='tf2_ros',executable='static_transform_publisher',arguments=['--frame-id','sentry/chassis','--child-frame-id','base_link'],parameters=[{'use_sim_time':True}]),
        Node(package='nav_simulation',executable='sim_adapter',parameters=[{'use_sim_time':True,'contact_config_verified':True}],output='screen'),
    ]


def generate_launch_description():
    args={'gui':'true','x':'29.325','y':'8.475','yaw':'0','real_time_factor':'1.0','partition':'trident-course'}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in args.items()]+[OpaqueFunction(function=start)])
