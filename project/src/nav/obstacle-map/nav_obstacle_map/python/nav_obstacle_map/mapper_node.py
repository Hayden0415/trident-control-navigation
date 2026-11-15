#!/usr/bin/env python3
"""ROS 封装已提供；算法位于 student_map.py。"""
from collections import deque
from dataclasses import replace
import math
import numpy as np
import rclpy
import signal
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_obstacle_map.student_map import GridSpec, project_cloud, inflate_grid


class Mapper(Node):
    def __init__(self):
        super().__init__('student_mapper')
        defaults = GridSpec()
        self.spec = GridSpec(**{name: self.declare_parameter(name, value).value
                               for name, value in vars(defaults).items()})
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.raw = self.create_publisher(OccupancyGrid, '/nav/map/raw', qos)
        self.inflated = self.create_publisher(OccupancyGrid, '/nav/map', qos)
        self.create_subscription(PointCloud2, '/nav/cloud', self.on_cloud, qos_profile_sensor_data)
        self.reported = False
        self.poses = deque(maxlen=200)
        self.last_cloud_ns = -1
        self.create_subscription(Odometry, '/nav/odom', self.on_odom, qos_profile_sensor_data)

    @staticmethod
    def stamp_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def on_odom(self, odom):
        if odom.header.frame_id == 'map':
            t = self.stamp_ns(odom.header.stamp)
            if self.poses and t < self.poses[-1][0]:
                self.poses.clear()  # simulation reset
                self.last_cloud_ns = -1
            p = odom.pose.pose.position
            if np.isfinite([p.x, p.y]).all():
                self.poses.append((t, p.x, p.y))

    def message(self, grid, header, spec):
        if (not isinstance(grid, np.ndarray) or grid.shape != (spec.height, spec.width)
                or grid.dtype != np.uint8 or not np.isin(grid, [0, 100]).all()):
            raise ValueError('algorithm output must be a binary uint8 grid of shape (height,width)')
        msg = OccupancyGrid()
        msg.header = header  # preserve cloud timestamp and actual frame
        msg.info.resolution = spec.resolution
        msg.info.width, msg.info.height = spec.width, spec.height
        msg.info.origin.position.x, msg.info.origin.position.y = spec.origin_x, spec.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel(order='C').astype(np.int8).tolist()
        return msg

    def on_cloud(self, cloud):
        if cloud.header.frame_id != 'map':
            self.get_logger().error('Point cloud must already be transformed into map frame')
            return
        stamp = self.stamp_ns(cloud.header.stamp)
        if stamp <= self.last_cloud_ns:
            return
        poses = [p for p in self.poses if 0 <= stamp-p[0] <= 250_000_000]
        if not poses:
            return  # no recent pose at/before this sensor timestamp
        _, x, y = max(poses)
        spec = replace(self.spec,
                       origin_x=math.floor(x/self.spec.resolution)*self.spec.resolution-self.spec.width*self.spec.resolution/2,
                       origin_y=math.floor(y/self.spec.resolution)*self.spec.resolution-self.spec.height*self.spec.resolution/2)
        try:
            points = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=False)
            points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
            raw = project_cloud(points, spec)
            inflated = inflate_grid(raw, spec)
            # Validate both before publishing either.
            raw_msg, inflated_msg = self.message(raw, cloud.header, spec), self.message(inflated, cloud.header, spec)
            self.raw.publish(raw_msg)
            self.inflated.publish(inflated_msg)
            self.last_cloud_ns = stamp
        except (NotImplementedError, ValueError, TypeError, AssertionError) as exc:
            if not self.reported:
                self.get_logger().error(str(exc))
                self.reported = True


def main():
    rclpy.init()
    node = Mapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
