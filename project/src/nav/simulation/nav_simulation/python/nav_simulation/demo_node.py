#!/usr/bin/env python3
"""Deterministic planar kinematic fixture, NOT a Gazebo/hardware simulation."""
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import math
import time
import numpy as np
import rclpy
import signal
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# World rectangles in metres: bounds and a central wall; go around its top/bottom.
OBSTACLES = [(-0.9, -0.7, -0.9, 6.9), (8.7, 8.9, -0.9, 6.9),
             (-0.9, 8.9, -0.9, -0.7), (-0.9, 8.9, 6.7, 6.9),
             (3.0, 3.4, -0.9, 3.7)]


class Demo(Node):
    def __init__(self):
        super().__init__('course_demo')
        self.scene = self.declare_parameter('scene', 'wall').value
        if self.scene not in ('wall', 'forest'):
            raise ValueError('scene must be wall or forest')
        self.pillars = []
        self.obstacles = OBSTACLES if self.scene == 'wall' else []
        if self.scene == 'forest':
            world = json.loads((Path(get_package_share_directory('nav_simulation'))/'resource/forest-world.json').read_text())
            self.pillars = world['pillars']
            self.bounds = world['bounds_xy']
        self.diagnostic = self.declare_parameter('diagnostic_grid', False).value
        self.auto_goal = self.declare_parameter('auto_goal', True).value
        self.cloud_pub = self.create_publisher(PointCloud2, '/nav/cloud', 1)
        self.odom_pub = self.create_publisher(Odometry, '/nav/odom', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/nav/goal', 1)
        self.collision_pub = self.create_publisher(Bool, '/course/collision', 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/nav/map', qos) if self.diagnostic else None
        self.create_subscription(Twist, '/nav/cmd_vel', self.cmd, 1)
        self.x = self.declare_parameter('start_x', 29.325 if self.scene == 'forest' else 0.5).value
        self.y = self.declare_parameter('start_y', 8.475 if self.scene == 'forest' else 0.5).value
        self.goal_x, self.goal_y = (1.475, 4.375) if self.scene == 'forest' else (7.5, 0.5)
        self.vx, self.vy = 0.0, 0.0
        self.u = (0.0, 0.0)
        self.last_cmd = time.monotonic()
        self.start = time.monotonic()
        self.goal_sent = False
        self.hit = False
        points = []
        for x0, x1, y0, y1 in self.obstacles:
            for x in np.arange(x0, x1 + 0.001, 0.04):
                for y in np.arange(y0, y1 + 0.001, 0.04):
                    points.append([x, y, 0.6])
        for pillar in self.pillars:
            n = max(32, math.ceil(2*math.pi*pillar['radius']/0.04))
            points.extend([[pillar['x']+pillar['radius']*math.cos(a),
                            pillar['y']+pillar['radius']*math.sin(a), 0.6]
                           for a in np.linspace(0, 2*math.pi, n, endpoint=False)])
        # Ground points must be rejected by the student height slice.
        points.extend([[float(x), float(y), 0.0] for x in np.arange(0, 8, 0.2)
                       for y in np.arange(0, 6, 0.2)])
        self.points = np.asarray(points, dtype=np.float32)
        self.sensor_range = 5.0
        self.create_timer(0.05, self.step)
        self.create_timer(0.2, self.map_tick)

    def cmd(self, msg):
        self.u = (msg.linear.x, msg.linear.y)
        self.last_cmd = time.monotonic()

    def map_tick(self):
        header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        # Synthetic range-limited scans, not a full world snapshot or a ray simulator.
        # Coordinates are already transformed to map, as on the real adapter.
        points = self.points[np.linalg.norm(self.points[:, :2]-[self.x, self.y], axis=1) <= self.sensor_range]
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points.tolist()))
        if self.grid_pub and self.scene != 'wall':
            raise ValueError('diagnostic grid is only available in wall fixture')
        if self.grid_pub:
            # Hand-authored navigation fixture; independent of student algorithms.
            # It intentionally supplies no raw map and does not count as student acceptance.
            grid = np.zeros((80, 100), dtype=np.int8)
            grid[:, :6] = 100
            grid[:, 94:] = 100
            grid[:6, :] = 100
            grid[74:, :] = 100
            grid[:50, 37:47] = 100
            msg = OccupancyGrid(header=header)
            msg.info.width, msg.info.height, msg.info.resolution = 100, 80, 0.1
            msg.info.origin.position.x = msg.info.origin.position.y = -1.0
            msg.info.origin.orientation.w = 1.0
            msg.data = grid.ravel().tolist()
            self.grid_pub.publish(msg)

    def step(self):
        tx, ty = self.u if time.monotonic()-self.last_cmd < 0.3 else (0.0, 0.0)
        self.vx += (tx-self.vx)*0.25
        self.vy += (ty-self.vy)*0.25
        nx, ny = self.x+self.vx*0.05, self.y+self.vy*0.05
        # Circular 0.22 m footprint against original rectangles.
        collided = any(math.hypot(nx-min(max(nx,a),b), ny-min(max(ny,c),d)) < 0.22
                       for a,b,c,d in self.obstacles)
        collided = collided or any(math.hypot(nx-p["x"], ny-p["y"]) < p["radius"]+0.22 for p in self.pillars)
        if self.scene == "forest":
            x0,x1,y0,y1 = self.bounds
            collided = collided or not (x0+0.22 <= nx <= x1-0.22 and y0+0.22 <= ny <= y1-0.22)
        if collided:
            self.hit = True
            self.vx = self.vy = 0.0
        else:
            self.x, self.y = nx, ny
        self.collision_pub.publish(Bool(data=self.hit))
        msg = Odometry()
        msg.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = self.vx, self.vy
        self.odom_pub.publish(msg)
        if self.auto_goal and not self.goal_sent and time.monotonic()-self.start > 2.0:
            goal = PoseStamped(header=msg.header)
            goal.pose.position.x, goal.pose.position.y = self.goal_x, self.goal_y
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.goal_sent = True


def main():
    rclpy.init()
    node = Demo()
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
