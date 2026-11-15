"""Generate a benchmark variant from the exact upstream Omni_sim robot and 25 UC mesh.

The versioned coaxial wheel correction follows conversion; OmniDrive topology is retained. Explicit profiles
select force PID or a torque-limited physical joint velocity motor. Sensor sampling,
a body collision envelope and removal of the free zero-mass base joint are explicit. No 2D dynamics are substituted for Gazebo physics.
"""
from pathlib import Path
import math
import subprocess
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory
from .forest import FOREST_MAP_IDS, load_layout
from .contact_model import finalize_world


def prepare(directory, x=3.4, y=9.5, yaw=0.0, z=0.4, seed=1, rate=1.0, wheel_control_mode="force_pid", map_id="rmuc_2025", wheel_velocity_limit_rad_s=35.0):
    if map_id not in ("rmuc_2025", *FOREST_MAP_IDS):
        raise ValueError("Unknown simulator map_id")
    if wheel_control_mode not in ("force_pid", "velocity_motor"):
        raise ValueError("wheel_control_mode must be force_pid or velocity_motor")
    if not math.isfinite(wheel_velocity_limit_rad_s) or wheel_velocity_limit_rad_s <= 0:
        raise ValueError("wheel_velocity_limit_rad_s must be finite and positive")
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    resources = Path(get_package_share_directory('nav_simulation'))
    own = Path(get_package_share_directory('nav_simulation'))
    robot_dir = resources / 'resource/models/sentry'
    doc = xacro.process_file(str(robot_dir / 'robot.urdf.xacro'), mappings={'use_force_commands': 'false'})
    robot = ET.fromstring(doc.toxml())
    for item in list(robot):
        if (item.tag == 'link' and item.get('name') == 'base_footprint') or (item.tag == 'joint' and item.get('name') == 'base_joint'):
            robot.remove(item)
    for mesh in robot.findall('.//mesh'):
        if mesh.get('filename', '').startswith('meshes/'):
            mesh.set('filename', 'model://sentry/' + mesh.get('filename'))
    chassis = robot.find("link[@name='chassis']")
    collision = ET.SubElement(chassis, 'collision', name='benchmark_body')
    ET.SubElement(collision, 'origin', xyz='0 0 0.13', rpy='0 0 0')
    geometry = ET.SubElement(collision, 'geometry')
    ET.SubElement(geometry, 'box', size='0.45 0.45 0.26')
    # The original 5000 x128 GPU scan is excessive for a reproducible CPU pilot.
    # Both planners receive this identical angular sampling and noiseless scene.
    for gazebo in robot.findall('gazebo'):
        for sensor in list(gazebo.findall('sensor')):
            if sensor.get('type') == 'camera':
                gazebo.remove(sensor)
                continue
            if sensor.get('type') == 'gpu_lidar':
                sensor.find('update_rate').text = '10'
                sensor.find('visualize').text = 'false'
                sensor.find('ray/scan/horizontal/samples').text = '1440'
                sensor.find('ray/scan/vertical/samples').text = '32'
                for noise in list(sensor.findall('noise')):
                    sensor.remove(noise)
    # Contact sensor is attached to the actual chassis collision envelope.
    gz_chassis = ET.SubElement(robot, 'gazebo', reference='chassis')
    contact = ET.SubElement(gz_chassis, 'sensor', name='benchmark_contact', type='contact')
    ET.SubElement(contact, 'always_on').text = 'true'
    ET.SubElement(contact, 'update_rate').text = '100'
    ET.SubElement(ET.SubElement(contact, 'contact'), 'collision').text = 'benchmark_body'
    urdf = ET.tostring(robot, encoding='unicode')
    urdf_path = directory / 'benchmark_robot.urdf'
    urdf_path.write_text(urdf)
    result = subprocess.run(['ign', 'sdf', '-p', str(urdf_path)], text=True, capture_output=True, check=True)
    sdf = ET.fromstring(result.stdout)
    model = sdf.find('model')
    model.set('name', 'sentry')
    # URDF conversion renames collisions when fixed links are lumped. Contact
    # sensor references must use the converted SDF name, not the URDF name.
    chassis_sdf = model.find("link[@name='chassis']")
    body_collisions = [c.get('name') for c in chassis_sdf.findall('collision')
                       if 'benchmark_body' in c.get('name', '')]
    if len(body_collisions) != 1:
        raise RuntimeError('Could not identify the converted benchmark body collision')
    chassis_sdf.find("sensor[@name='benchmark_contact']/contact/collision").text = body_collisions[0]
    # Keep the robot visible to the GUI while excluding its own CAD meshes from
    # lidar rays. The upstream inverted sensor otherwise mostly sees itself.
    for visual in model.findall('.//visual'):
        ET.SubElement(visual, 'visibility_flags').text = '2'
    for sensor in model.findall('.//sensor'):
        if sensor.get('name') == 'lidar_sensor':
            ray = sensor.find('lidar')
            if ray is None:
                ray = sensor.find('ray')
            ET.SubElement(ray, 'visibility_mask').text = '1'
    for plugin in model.findall('plugin'):
        if plugin.get('filename') == 'OmniDrive':
            ET.SubElement(plugin, 'benchmark_seed').text = str(seed)
            ET.SubElement(plugin, 'wheel_control_mode').text = wheel_control_mode
            ET.SubElement(plugin, 'wheel_velocity_limit').text = str(wheel_velocity_limit_rad_s)
            plugin.find('wheel_radius').text = '0.10'  # actual upstream wheel collision cylinder radius
            ET.SubElement(plugin, 'wheel_axis_projection').text = '0.7071067811865476'
            ET.SubElement(plugin, 'wheel_pid_p').text = '0.05'
            ET.SubElement(plugin, 'wheel_pid_i').text = '0.10'
            ET.SubElement(plugin, 'wheel_integral_torque_limit').text = '0.20'
            ET.SubElement(plugin, 'wheel_integrator_reset_on_stop').text = 'true'
            ET.SubElement(plugin, 'wheel_torque_limit').text = '10.0'
    pose = model.find('pose')
    if pose is None:
        pose = ET.SubElement(model, 'pose')
    pose.text = f'{x} {y} {z} 0 0 {yaw}'
    # Finite maximum wheel speed is declared explicitly for both branches.
    for joint in model.findall('joint'):
        limit = joint.find('axis/limit/velocity')
        if limit is not None and 'wheel' in joint.get('name', ''):
            limit.text = str(wheel_velocity_limit_rad_s)
    world = ET.parse(own / 'resource/worlds/forest_template.sdf')
    node = world.getroot().find('world')
    if map_id in FOREST_MAP_IDS:
        from .forest import populate
        populate(node, load_layout(own / 'resource/worlds', map_id))
    physics = ET.SubElement(node, 'physics', name='benchmark', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_factor').text = str(rate)
    ET.SubElement(physics, 'real_time_update_rate').text = '0'
    ET.SubElement(node, 'plugin', filename='ignition-gazebo-contact-system', name='ignition::gazebo::systems::Contact')
    node.append(model)
    world_path = directory / (map_id + '_benchmark.sdf')
    world.write(world_path, encoding='utf-8', xml_declaration=True)
    # Last SDF mutation: another ign sdf -p would round these constants again.
    finalize_world(world_path)
    return urdf, str(world_path), str(own / 'resource/models')
