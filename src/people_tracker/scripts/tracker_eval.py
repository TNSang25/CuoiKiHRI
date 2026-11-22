#!/usr/bin/env python3
"""Compare tracked people (/people) with simulator ground truth (/human_states).

Each ground-truth person within --range of the robot is matched to the nearest track within
--match metres. Reports recall, false tracks per frame, position and velocity RMSE.
People hidden from the lidar (e.g. behind a corner) count as misses: this is the real-world view.
"""
import argparse
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from social_nav_msgs.msg import Agents, People


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60.0, help='Wall seconds to record')
    parser.add_argument('--match', type=float, default=0.5)
    parser.add_argument('--range', type=float, default=6.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('tracker_eval')
    state = {'gt': None, 'robot': None}
    stats = {'frames': 0, 'gt': 0, 'matched': 0, 'false': 0, 'pos_se': 0.0, 'vel_se': 0.0}

    def on_tracks(msg):
        if state['gt'] is None or state['robot'] is None:
            return
        rx, ry = state['robot']
        gt = [a for a in state['gt'].agents
              if math.dist((a.position.position.x, a.position.position.y), (rx, ry)) <= args.range]
        tracks = list(msg.people)
        used = set()
        stats['frames'] += 1
        for a in gt:
            gx, gy = a.position.position.x, a.position.position.y
            best, best_d = None, args.match
            for i, t in enumerate(tracks):
                d = math.dist((t.position.x, t.position.y), (gx, gy))
                if i not in used and d < best_d:
                    best, best_d = i, d
            stats['gt'] += 1
            if best is not None:
                used.add(best)
                t = tracks[best]
                stats['matched'] += 1
                stats['pos_se'] += best_d ** 2
                stats['vel_se'] += (t.velocity.x - a.velocity.linear.x) ** 2 + (t.velocity.y - a.velocity.linear.y) ** 2
        stats['false'] += len(tracks) - len(used)

    node.create_subscription(Agents, 'human_states', lambda m: state.__setitem__('gt', m), 10)
    node.create_subscription(Odometry, 'odom', lambda m: state.__setitem__(
        'robot', (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
    node.create_subscription(People, 'people', on_tracks, 10)
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    m = max(stats['matched'], 1)
    result = {
        'frames': stats['frames'],
        'recall': round(stats['matched'] / max(stats['gt'], 1), 3),
        'false_tracks_per_frame': round(stats['false'] / max(stats['frames'], 1), 3),
        'position_rmse_m': round(math.sqrt(stats['pos_se'] / m), 3),
        'velocity_rmse_mps': round(math.sqrt(stats['vel_se'] / m), 3),
    }
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
