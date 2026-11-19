#!/usr/bin/env python3
"""Check live Gazebo time, robot state and movement of the configured people."""
import json
import argparse
import os
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from bot_interfaces.msg import Agent, Agents


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--agents', type=int, default=3)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--output', type=Path, default=Path(os.environ.get("BOT_RUNTIME_DIR", ".runtime")) / "smoke-test.json")
    args = parser.parse_args()
    if args.agents < 1 or args.duration <= 0 or args.timeout <= args.duration:
        parser.error('agents/duration must be positive and timeout must exceed duration')
    rclpy.init()
    node = Node('bot_simulation_check')
    first, last, counts = {}, {}, {'human': 0, 'robot': 0, 'clock': 0}
    max_displacement = {}
    last_seen = {'human': 0.0, 'robot': 0.0, 'clock': 0.0}
    invalid_positions = []
    heading_errors = []
    clocks = []

    def humans(msg):
        counts['human'] += 1
        last_seen['human'] = time.monotonic()
        for agent in msg.agents:
            position = agent.position.position
            xy = (position.x, position.y)
            if not all(math.isfinite(value) for value in xy):
                invalid_positions.append(agent.id)
                continue
            first.setdefault(agent.id, xy)
            last[agent.id] = xy
            max_displacement[agent.id] = max(max_displacement.get(agent.id, 0.0),
                                             math.dist(first[agent.id], xy))
            vx, vy = agent.velocity.linear.x, agent.velocity.linear.y
            if math.hypot(vx, vy) > 0.1:
                difference = agent.yaw - math.atan2(vy, vx)
                heading_errors.append(abs(math.degrees(math.atan2(math.sin(difference),
                                                                  math.cos(difference)))))

    def robot(msg):
        counts['robot'] += 1
        last_seen['robot'] = time.monotonic()

    def clock(msg):
        counts['clock'] += 1
        last_seen['clock'] = time.monotonic()
        stamp = msg.clock.sec + msg.clock.nanosec / 1e9
        if not clocks:
            clocks.extend([stamp, stamp])
        clocks[-1] = stamp

    subscriptions = [
        node.create_subscription(Agents, '/human_states', humans, 10),
        node.create_subscription(Agent, '/robot_states', robot, 10),
        node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data),
    ]
    deadline = time.monotonic() + args.timeout
    sampling_start = None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if len(first) == args.agents and sampling_start is None:
                sampling_start = time.monotonic()
            if sampling_start is not None and time.monotonic() - sampling_start >= args.duration:
                break
        displacement = {str(i): round(math.dist(first[i], xy), 4) for i, xy in last.items()}
        sorted_errors = sorted(heading_errors)
        heading_p95 = sorted_errors[int((len(sorted_errors) - 1) * 0.95)] if sorted_errors else None
        result = {
            'messages': counts, 'agent_count': len(first),
            'simulation_seconds': round(clocks[-1] - clocks[0], 3) if clocks else 0,
            'displacement_m': displacement,
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
            'max_displacement_m': {str(i): round(value, 4) for i, value in max_displacement.items()},
            'streams_active': all(time.monotonic() - stamp < 2.0 for stamp in last_seen.values()),
            'invalid_positions': invalid_positions,
            'heading_error_degrees': {
                'samples': len(sorted_errors),
                'p95': round(heading_p95, 4) if heading_p95 is not None else None,
                'max': round(max(sorted_errors), 4) if sorted_errors else None,
            },
            'first_positions': first, 'last_positions': last,
        }
        passed = (len(first) == args.agents and counts['robot'] > 1 and
                  result['simulation_seconds'] > 0.5 and result['streams_active'] and
                  not invalid_positions and
                  heading_p95 is not None and heading_p95 < 5.0 and
                  all(distance > 0.1 for distance in max_displacement.values()))
        result['passed'] = passed
        output = args.output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
