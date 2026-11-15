#!/usr/bin/env python3
"""Run real ROS nodes over DDS; verify arrival, clearance and stale-input stop."""
import argparse
import math
import os
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool, String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--diagnostic', action='store_true', help='Bypass student mapper; supplied stack only')
    args = parser.parse_args()
    rclpy.init()
    node = Node('course_acceptance')
    state = dict(arrived=False, collision=False, x=0.5, y=0.5, raw=False,
                 inflated=False, last_cmd=None, cmd_received=0.0, max_speed=0.0)
    def odom(m):
        state.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y)
    def cmd(m):
        v = math.hypot(m.linear.x, m.linear.y)
        state.update(last_cmd=v, cmd_received=time.monotonic(), max_speed=max(v,state['max_speed']))
    def status(m):
        print('navigation:', m.data, flush=True)
        if m.data == 'goal_reached':
            state['arrived'] = True
    node.create_subscription(Odometry, '/nav/odom', odom, 10)
    node.create_subscription(Twist, '/nav/cmd_vel', cmd, 10)
    node.create_subscription(Bool, '/course/collision', lambda m: state.update(collision=m.data or state['collision']), 10)
    node.create_subscription(String, '/nav/control/status', status, 10)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, '/nav/map/raw', lambda m: state.update(raw=True), qos)
    node.create_subscription(OccupancyGrid, '/nav/map', lambda m: state.update(inflated=True), qos)
    processes=[]
    def start(package, exe, *params):
        p=subprocess.Popen(['ros2','run',package,exe,*params],start_new_session=True)
        processes.append(p)
        return p
    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGTERM)
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
    def spin_for(seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.05)
    try:
        start('nav_planning','planner_node')
        start('nav_legacy_control','controller_node')
        if not args.diagnostic:
            start('nav_obstacle_map','student_mapper')
        demo=start('nav_simulation','course_demo','--ros-args','-p',f'diagnostic_grid:={str(args.diagnostic).lower()}')
        started=time.monotonic()
        deadline=started+110
        while time.monotonic()<deadline and not state['arrived']:
            rclpy.spin_once(node,timeout_sec=0.05)
            if state['collision']: raise RuntimeError('footprint collision')
            if any(p.poll() is not None for p in processes): raise RuntimeError('node exited')
        if not state['arrived']: raise RuntimeError(f'arrival timeout: {state}')
        arrival_elapsed=time.monotonic()-started
        arrival_error=math.hypot(state['x']-7.5,state['y']-0.5)
        if math.hypot(state['x']-7.5,state['y']-0.5)>0.2: raise RuntimeError('false arrival status')
        if not state['inflated']: raise RuntimeError('no grid received')
        if not args.diagnostic and not state['raw']: raise RuntimeError('no student raw grid')
        if state['max_speed']>0.65001: raise RuntimeError('speed bound exceeded')
        # Start another goal, stop the source while robot is moving, then verify stop.
        from geometry_msgs.msg import PoseStamped
        goal_pub=node.create_publisher(PoseStamped,'/nav/goal',10)
        spin_for(0.5)
        goal=PoseStamped();goal.header.frame_id='map';goal.pose.position.x=0.5;goal.pose.position.y=0.5;goal.pose.orientation.w=1.0
        for _ in range(3): goal_pub.publish(goal);spin_for(0.1)
        spin_for(1)
        if state['last_cmd'] is None or state['last_cmd']<0.01: raise RuntimeError('return goal did not move')
        stop(demo)
        spin_for(1.5)
        if state['last_cmd'] is None or state['last_cmd']>1e-9: raise RuntimeError('stale-input stop failed')
        if time.monotonic()-state['cmd_received']>0.3: raise RuntimeError('controller stopped publishing instead of commanding zero')
        if state['collision']: raise RuntimeError('collision observed')
        label='SUPPLIED NAVIGATION ONLY (student work bypassed)' if args.diagnostic else 'STUDENT CLOSED LOOP'
        print(f'PASS: {label}; arrival={arrival_elapsed:.2f}s error={arrival_error:.4f}m; no collision; max_speed={state["max_speed"]:.4f}m/s; stale-input stop',flush=True)
    finally:
        for p in reversed(processes): stop(p)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
