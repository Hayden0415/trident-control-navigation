#!/usr/bin/env python3
"""Provided half-world prior plus student live local-grid fusion."""
import json
import signal
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav_obstacle_map.global_prior import prior_grid, overlay_local


class Prior(Node):
    def __init__(self):
        super().__init__('global_prior')
        path = self.declare_parameter('prior_map', '').value
        self.spec = json.loads(Path(path).read_text())
        self.prior = prior_grid(self.spec)
        self.last_stamp = -1
        qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.prior_pub = self.create_publisher(OccupancyGrid,'/nav/map/prior',qos)
        self.fused_pub = self.create_publisher(OccupancyGrid,'/nav/map',qos)
        self.create_subscription(OccupancyGrid,'/nav/map/local',self.on_local,qos)
        self.create_timer(0.2,self.publish_prior)
        self.get_logger().info(f'Loaded {len(self.spec["pillars"])} known pillars; navigating on prior only until the first valid local map')

    def message(self, data, stamp):
        m = OccupancyGrid()
        m.header.frame_id = 'map'; m.header.stamp = stamp
        m.info.width,m.info.height = self.spec['width'],self.spec['height']
        m.info.resolution = self.spec['resolution']
        m.info.origin.position.x,m.info.origin.position.y = self.spec['origin_xy']
        m.info.origin.orientation.w = 1.
        m.data = data.astype(np.int8).ravel().tolist()
        return m

    def publish_prior(self):
        # Allow the unfinished assignment to navigate using only the partial prior.
        # Once local mapping works, retain its timestamps so a stalled mapper
        # still causes the planner/controller freshness checks to stop the robot.
        m = self.message(self.prior,self.get_clock().now().to_msg())
        self.prior_pub.publish(m)
        if self.last_stamp < 0:
            self.fused_pub.publish(m)

    def on_local(self,m):
        stamp=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
        if stamp <= self.last_stamp:
            return
        try:
            q=m.info.origin.orientation
            if m.header.frame_id!='map' or abs(q.x)+abs(q.y)+abs(q.z)>1e-6 or abs(q.w-1)>1e-6:
                raise ValueError('expected axis-aligned local map in map frame')
            if m.info.width*m.info.height>250000 or m.info.width<=0 or m.info.height<=0:
                raise ValueError('invalid local dimensions')
            local=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
            fused=overlay_local(self.prior,self.spec,local,
                                (m.info.origin.position.x,m.info.origin.position.y),m.info.resolution)
            self.fused_pub.publish(self.message(fused,m.header.stamp))
            if self.last_stamp < 0:
                self.get_logger().info('Local map received; navigation now uses prior plus live local obstacles')
            self.last_stamp=stamp
        except (ValueError,TypeError) as exc:
            self.get_logger().error(str(exc))


def main():
    rclpy.init();node=Prior()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        signal.signal(signal.SIGTERM,signal.SIG_IGN)
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
