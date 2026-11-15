#!/usr/bin/env python3
"""DDS acceptance: changing scans, rolling origin, stale and out-of-order inputs."""
import os
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def main():
    rclpy.init()
    node = Node('live_map_acceptance')
    cloud_pub = node.create_publisher(PointCloud2, '/nav/cloud', 10)
    odom_pub = node.create_publisher(Odometry, '/nav/odom', 10)
    maps = {}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/nav/map', lambda m: maps.update(inflated=m), qos)
    node.create_subscription(OccupancyGrid, '/nav/map/raw', lambda m: maps.update(raw=m), qos)
    proc = subprocess.Popen(['ros2', 'run', 'nav_obstacle_map', 'student_mapper'], start_new_session=True)

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if proc.poll() is not None:
                raise RuntimeError('mapper exited')

    def scan(points, x=0., y=0.):
        # Repeat pose before scan to exercise transport independently of arrival order.
        stamp = node.get_clock().now().to_msg()
        h = Header(frame_id='map', stamp=stamp)
        odom = Odometry(header=h, child_frame_id='base_link')
        odom.pose.pose.orientation.w = 1.
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom_pub.publish(odom); spin(.08)
        cloud = point_cloud2.create_cloud_xyz32(h, points)
        cloud_pub.publish(cloud)
        deadline = time.monotonic()+3
        while time.monotonic()<deadline:
            spin(.02)
            if all(k in maps and maps[k].header.stamp == stamp for k in ('raw','inflated')):
                return cloud
        raise RuntimeError('no map preserving scan timestamp (TODO or synchronization error)')

    def value(m, x, y):
        import math
        ix = math.floor((x-m.info.origin.position.x)/m.info.resolution)
        iy = math.floor((y-m.info.origin.position.y)/m.info.resolution)
        return m.data[ix+iy*m.info.width]

    try:
        spin(2)
        old = scan([[1.05, .05, .6]])
        assert value(maps['raw'], 1.05, .05) == 100
        assert value(maps['inflated'], 1.15, .05) == 100
        scan([[2.05, .05, .6]])
        assert value(maps['inflated'], 1.05, .05) == 0, 'old obstacle retained'
        assert value(maps['raw'], 2.05, .05) == 100
        scan([])
        assert all(v == 0 for v in maps['inflated'].data), 'empty scan did not clear obstacles'
        cloud_pub.publish(old); spin(.2)
        assert all(v == 0 for v in maps['inflated'].data), 'old scan replaced newer map'
        origin = maps['raw'].info.origin.position.x
        scan([[3.05, .05, .6]], x=2.)
        assert abs(maps['raw'].info.origin.position.x-origin-2.) < .001
        assert value(maps['raw'], 3.05, .05) == 100
        stamp = maps['raw'].header.stamp
        spin(.4)
        assert maps['raw'].header.stamp == stamp, 'mapper fabricated fresh input'
        print('PASS: live scan appearance/movement/disappearance, inflation, rolling origin, timestamps, old-scan rejection', flush=True)
    finally:
        os.killpg(proc.pid, signal.SIGTERM)
        try: proc.wait(timeout=5)
        except subprocess.TimeoutExpired: os.killpg(proc.pid, signal.SIGKILL); proc.wait()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
