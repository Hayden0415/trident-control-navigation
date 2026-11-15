#!/usr/bin/env python3
"""ROS adapters for Gazebo truth, native RogMap projection and cloud coordinates."""
from collections import deque
from copy import deepcopy
import math
import struct
import json
import os
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Bool, String
from ros_gz_interfaces.msg import Contacts
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


def rotation(q):
    v = np.array([q.x, q.y, q.z, q.w], dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError('invalid zero quaternion')
    x, y, z, w = v / n
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class Adapter(Node):
    def __init__(self):
        super().__init__('nav_sim_adapter')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/nav/odom', 10)
        self.map_odom_pub = self.create_publisher(Odometry, '/nav/mapping_odom', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/nav/cloud', 2)
        self.cmd_pub = self.create_publisher(Twist, '/nav/sim/cmd_vel', 5)
        self.create_subscription(Odometry, '/nav/sim/odom_raw', self.odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/nav/sim/cloud_raw', self.pending_cloud, qos_profile_sensor_data)
        self.create_subscription(Twist, '/nav/cmd_vel', self.command, 5)
        self.pending = deque(maxlen=5)
        self.create_timer(0.01, self.transform_pending)
        self.last_cmd_stamp = None
        self.create_timer(0.05, self.watchdog)
        self.transformed = 0
        self.contact_configured = self.declare_parameter('contact_config_verified', False).value
        self.contact_topic = '/world/default/model/sentry/link/chassis/sensor/benchmark_contact/contact'
        self.collision_pub = self.create_publisher(Bool, '/nav/collision', 10)
        self.collision_status_pub = self.create_publisher(String, '/nav/collision_status', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Contacts, '/nav/sim/contacts', self.contacts, 100)
        self.contact_rx = 0
        self.contact_positive_rx = 0
        self.contact_last_wall = None
        self.contact_last_sim_s = None
        self.collision_latched = False
        self.truth_last_wall = None
        self.truth_progress_wall = None
        self.truth_last_sim_ns = None
        self.contact_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='nav-contact-health')
        self.contact_future = None
        self.contact_probe_started = -1e9
        self.contact_probe_finished = None
        self.contact_source_discovered = False
        self.contact_probe_error = None
        self.create_timer(0.2, self.contact_health)
        self.contact_health()

    def odom(self, msg):
        wall = time.monotonic()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.truth_last_sim_ns is None or stamp > self.truth_last_sim_ns:
            self.truth_progress_wall = wall
        self.truth_last_sim_ns = stamp
        self.truth_last_wall = wall
        # OmniDrive publishes actual WorldPose and local-frame velocity.
        out = deepcopy(msg)
        out.header.frame_id = 'map'
        out.child_frame_id = 'base_link'
        self.odom_pub.publish(out)
        transform = TransformStamped()
        transform.header = out.header
        transform.child_frame_id = 'sentry/chassis'
        transform.transform.translation.x = out.pose.pose.position.x
        transform.transform.translation.y = out.pose.pose.position.y
        transform.transform.translation.z = out.pose.pose.position.z
        transform.transform.rotation = out.pose.pose.orientation
        self.broadcaster.sendTransform(transform)

    def contacts(self, msg):
        self.contact_rx += 1
        self.contact_last_wall = time.monotonic()
        self.contact_last_sim_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg.contacts:
            self.contact_positive_rx += 1
            first_positive = not self.collision_latched
            self.collision_latched = True
            # Immediate actual-positive event; do not wait for the health probe.
            if first_positive:
                self.collision_pub.publish(Bool(data=True))
                self.contact_health()

    def probe_contact_source(self):
        try:
            result = subprocess.run(['ign', 'topic', '-i', '-t', self.contact_topic],
                                    env=os.environ.copy(), capture_output=True, text=True, timeout=3)
            discovered = (result.returncode == 0 and 'Publishers [' in result.stdout
                          and 'ignition.msgs.Contacts' in result.stdout)
            return discovered, None if discovered else result.stdout.strip()[:200]
        except (OSError, subprocess.TimeoutExpired) as exc:
            return False, str(exc)[:200]

    def contact_health(self):
        wall = time.monotonic()
        if self.contact_future is not None and self.contact_future.done():
            self.contact_source_discovered, self.contact_probe_error = self.contact_future.result()
            self.contact_probe_finished = wall
            self.contact_future = None
        if self.contact_future is None and wall - self.contact_probe_started >= 2.0:
            self.contact_probe_started = wall
            self.contact_future = self.contact_executor.submit(self.probe_contact_source)
        source_age = None if self.contact_probe_finished is None else wall - self.contact_probe_finished
        odom_age = None if self.truth_last_wall is None else wall - self.truth_last_wall
        progress_age = None if self.truth_progress_wall is None else wall - self.truth_progress_wall
        bridge_publishers = self.count_publishers('/nav/sim/contacts')
        healthy = bool(self.contact_configured and self.contact_source_discovered and
                       source_age is not None and source_age < 5.0 and bridge_publishers > 0 and
                       odom_age is not None and odom_age < 0.5 and
                       progress_age is not None and progress_age < 0.5)
        status = dict(schema_version=1, source='gazebo_contact_event_sensor',
                      observation_mode='event_absence_with_source_health', healthy=healthy,
                      configured=bool(self.contact_configured), source_topic=self.contact_topic,
                      source_discovered=self.contact_source_discovered,
                      source_discovery_age_wall_s=source_age, ros_bridge_publishers=bridge_publishers,
                      odom_age_wall_s=odom_age, sim_progress_age_wall_s=progress_age,
                      raw_messages_received=self.contact_rx,
                      raw_nonempty_messages_received=self.contact_positive_rx,
                      last_raw_age_wall_s=None if self.contact_last_wall is None else wall-self.contact_last_wall,
                      last_raw_sim_s=self.contact_last_sim_s, collision_latched=self.collision_latched,
                      negative_observation_kind=('actual_contact_latched' if self.collision_latched else
                                                 'healthy_event_source_absence' if healthy else 'unknown'),
                      stamp_sim_s=self.get_clock().now().nanoseconds*1e-9,
                      discovery_error=self.contact_probe_error)
        self.collision_status_pub.publish(String(data=json.dumps(status)))
        # False is an explicitly qualified event-source absence observation,
        # never a claim that an empty Contacts frame was received.
        if self.collision_latched or healthy:
            self.collision_pub.publish(Bool(data=self.collision_latched))

    def command(self, msg):
        if not all(math.isfinite(v) for v in [msg.linear.x, msg.linear.y, msg.angular.z]):
            self.get_logger().error('Rejected non-finite body velocity')
            return
        self.last_cmd_stamp = self.get_clock().now().nanoseconds
        self.cmd_pub.publish(msg)

    def watchdog(self):
        if self.last_cmd_stamp is not None and self.get_clock().now().nanoseconds - self.last_cmd_stamp > 300_000_000:
            self.cmd_pub.publish(Twist())
            self.last_cmd_stamp = None

    def pending_cloud(self, msg):
        self.pending.append(msg)

    def transform_pending(self):
        if not self.pending:
            return
        msg = self.pending[0]
        try:
            tf = self.buffer.lookup_transform('map', 'sentry/lidar_link', Time.from_msg(msg.header.stamp))
        except Exception:
            # Allow truth odometry and joint-state TF for the SAME sensor stamp
            # to arrive; never transform a cloud with an arbitrary latest pose.
            age = self.get_clock().now().nanoseconds - Time.from_msg(msg.header.stamp).nanoseconds
            if age > 500_000_000:
                self.pending.popleft()
            return
        self.pending.popleft()
        fields = {field.name: field for field in msg.fields}
        if not all(k in fields and fields[k].datatype == PointField.FLOAT32 for k in ['x', 'y', 'z']):
            self.get_logger().error('Expected float32 XYZ Gazebo lidar fields')
            return
        endian = '>' if msg.is_bigendian else '<'
        dtype = np.dtype({'names': ['x','y','z'], 'formats': [endian+'f4']*3,
                          'offsets': [fields[k].offset for k in ['x','y','z']], 'itemsize': msg.point_step})
        rows = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                          strides=(msg.row_step, msg.point_step))
        xyz = np.stack([rows[k].reshape(-1) for k in ['x','y','z']], axis=-1)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        tr = tf.transform.translation
        rr = rotation(tf.transform.rotation)
        # Sensor pose in the upstream URDF is (0,0,.2406, roll=pi).
        # Gazebo merges the fixed lidar link into yaw_link, so its raw frame ID
        # is not sufficient to recover this non-identity sensor offset.
        sensor_origin = rr @ np.array([0., 0., .2406]) + np.array([tr.x,tr.y,tr.z])
        sensor_rotation = rr @ np.diag([1., -1., -1.])
        world = (xyz @ sensor_rotation.T + sensor_origin).astype('<f4')
        # GPU lidar sees the robot's own wheel/chassis visuals. Remove its
        # conservative footprint from both live and regenerated sensor inputs.
        world = world[np.linalg.norm(world[:, :2] - sensor_origin[:2], axis=1) >= 0.35]
        xyzi = np.column_stack([world, np.zeros(len(world), dtype='<f4')]).astype('<f4')
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.header.frame_id = 'map'
        cloud.height = 1
        cloud.width = len(world)
        cloud.fields = [PointField(name=k, offset=4*i, datatype=PointField.FLOAT32, count=1) for i,k in enumerate(['x','y','z','intensity'])]
        cloud.point_step = 16
        cloud.row_step = 16*cloud.width
        cloud.is_dense = True
        cloud.data = xyzi.tobytes()
        odom = Odometry()
        odom.header = cloud.header
        odom.child_frame_id = 'nav_lidar_sensor'
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, sensor_origin)
        # q_sensor = q_lidar_link * q_roll(pi), not merely the link orientation.
        q = tf.transform.rotation
        odom.pose.pose.orientation.x = q.w
        odom.pose.pose.orientation.y = q.z
        odom.pose.pose.orientation.z = -q.y
        odom.pose.pose.orientation.w = -q.x
        self.map_odom_pub.publish(odom)
        self.cloud_pub.publish(cloud)
        self.transformed += 1
        if self.transformed == 1:
            self.get_logger().info(f'First global cloud: {len(world)} points; ray origin={sensor_origin.tolist()}')


def main(args=None):
    rclpy.init(args=args)
    node = Adapter()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.contact_executor.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
