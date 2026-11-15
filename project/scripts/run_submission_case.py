#!/usr/bin/env python3
"""Container-side Gazebo acceptance case; called by submit.py, not a manual launch."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', required=True)
    ap.add_argument('--timeout', type=float, default=180.)
    ap.add_argument('--goal-x', type=float, default=1.475)
    ap.add_argument('--goal-y', type=float, default=4.375)
    args = ap.parse_args()
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('submission_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
    counts = {k: 0 for k in ('odom', 'cloud', 'map', 'local', 'prior', 'path')}
    state = {'position': None, 'collision': False, 'contact': {}, 'path_points': 0, 'status': ''}
    last_seen = {}; procs = []; logs = []
    result = {'schema_version': 1, 'outcome': 'error', 'goal': [args.goal_x, args.goal_y],
              'timeout_sim_s': args.timeout, 'full_field_goal': [args.goal_x, args.goal_y] == [1.475, 4.375]}
    trajectory = (out/'trajectory.jsonl').open('w')
    def update(kind, msg):
        counts[kind] += 1; last_seen[kind] = time.monotonic()
        if kind == 'odom':
            p = msg.pose.pose.position
            state['position'] = [p.x, p.y, p.z]
            if counts[kind] % 10 == 0:
                trajectory.write(json.dumps({'sim_s': node.get_clock().now().nanoseconds/1e9, 'position': state['position']})+'\n')
        if kind == 'path': state['path_points'] = len(msg.poses)
    sensor = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
    grid = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs = [node.create_subscription(typ, topic, lambda m, k=kind: update(k, m), qos)
            for kind, typ, topic, qos in [
                ('odom', Odometry, '/nav/odom', sensor), ('cloud', PointCloud2, '/nav/cloud', sensor),
                ('map', OccupancyGrid, '/nav/map', grid), ('local', OccupancyGrid, '/nav/map/local', grid),
                ('prior', OccupancyGrid, '/nav/map/prior', grid), ('path', NavPath, '/nav/planning/path', 10)]]
    def contact(m):
        try: state['contact'] = json.loads(m.data); last_seen['contact'] = time.monotonic()
        except (ValueError, TypeError): pass
    subs += [node.create_subscription(Bool, '/nav/collision', lambda m: state.update(collision=state['collision'] or m.data), 10),
             node.create_subscription(String, '/nav/collision_status', contact, grid),
             node.create_subscription(String, '/nav/control/status', lambda m: state.update(status=m.data), 10)]
    goal = node.create_publisher(PoseStamped, '/nav/goal', 10)
    def spin():
        rclpy.spin_once(node, timeout_sec=.05)
        if any(p.poll() is not None for p in procs): raise RuntimeError('a module launch exited; inspect module logs')
    try:
        entries = [('environment', ['nav_simulation', 'environment.launch.py', 'partition:=trident-submission-'+str(os.getpid())]),
                   ('mapping', ['nav_obstacle_map', 'mapping.launch.py', 'use_sim_time:=true', 'use_global_prior:=true']),
                   ('navigation', ['nav_bringup', 'navigation.launch.py', 'use_sim_time:=true']),
                   ('rviz', ['nav_bringup', 'rviz.launch.py'])]
        for name, cmd in entries:
            log = (out/(name+'.log')).open('w'); logs.append(log)
            procs.append(subprocess.Popen(['ros2', 'launch']+cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic()+75
        while not (all(counts[k] > 1 for k in ('odom', 'cloud', 'map', 'prior')) and goal.get_subscription_count() >= 2):
            spin()
            if time.monotonic() > deadline: raise RuntimeError('startup timeout: clock/sensors/map/goal subscriptions not ready')
        (out/'ready.json').write_text(json.dumps({'counts': counts, 'position': state['position']}))
        deadline = time.monotonic()+60
        while not (out/'start.signal').exists():
            spin()
            if time.monotonic() > deadline: raise RuntimeError('host recorder did not signal start')
        g = PoseStamped(); g.header.frame_id = 'map'; g.header.stamp = node.get_clock().now().to_msg()
        g.pose.position.x = args.goal_x; g.pose.position.y = args.goal_y; g.pose.orientation.w = 1.
        goal.publish(g)
        sim_start = node.get_clock().now().nanoseconds/1e9; wall_start = time.monotonic()
        result['start_position'] = state['position']; reached_since = None
        while True:
            spin(); now = time.monotonic(); elapsed = node.get_clock().now().nanoseconds/1e9-sim_start
            pos = state['position']; error = math.hypot(pos[0]-args.goal_x, pos[1]-args.goal_y)
            if state['collision']:
                result['outcome'] = 'collision'; break
            if error < .18:
                reached_since = reached_since or now
                if now-reached_since > .5:
                    result['outcome'] = 'goal_reached'; break
            else: reached_since = None
            if elapsed >= args.timeout or now-wall_start > max(60., args.timeout*3):
                result['outcome'] = 'timeout'; break
            if any(now-last_seen.get(k, 0) > 4 for k in ('odom', 'cloud', 'map')):
                result['outcome'] = 'stale_sensor_or_map'; break
        result.update(elapsed_sim_s=elapsed, elapsed_wall_s=now-wall_start, final_error_m=error)
        # Keep the final position/map visible briefly in the recording.
        deadline = time.monotonic()+2
        while time.monotonic() < deadline: spin()
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        result.update(counts=counts, final_state=state)
        result['contact_observation_healthy'] = bool(state['contact'].get('healthy')) and time.monotonic()-last_seen.get('contact', 0) < 3
        result['passed'] = (result['outcome'] == 'goal_reached' and counts['local'] >= 5
                            and result['contact_observation_healthy'] and not state['collision'])
        for p in procs:
            if p.poll() is None: os.killpg(p.pid, signal.SIGINT)
        for p in procs:
            try: p.wait(timeout=8)
            except subprocess.TimeoutExpired: os.killpg(p.pid, signal.SIGKILL); p.wait()
        trajectory.close()
        for f in logs: f.close()
        node.destroy_node(); rclpy.try_shutdown()
        (out/'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
        if os.geteuid() == 0 and 'SUBMIT_UID' in os.environ:
            for path in [out, *out.rglob('*')]:
                try: os.chown(path, int(os.environ['SUBMIT_UID']), int(os.environ['SUBMIT_GID']), follow_symlinks=False)
                except FileNotFoundError: pass
    return 0 if result['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
