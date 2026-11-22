#!/usr/bin/env python3
"""Drive the robot from the level's START to its GOAL with Nav2 and summarise the run.

Sends one NavigateToPose goal once Nav2 is up and writes a JSON summary to $SIM_RESULTS_DIR:
success, time, path length, closest approach to a person, personal-space intrusions, collisions,
stops, mean accelerations and the distance at which the robot started to avoid.
"""
import argparse
import json
import math
import os
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

from social_nav_msgs.msg import Agents

PERSONAL_SPACE = 1.2   # m between robot and person centres (Hall's personal distance)


class Mission(Node):
    def __init__(self, args, level):
        super().__init__('robot_mission')
        self.args, self.level = args, level
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pose = None
        self.path_length = 0.0
        self.min_clearance = math.inf
        self.min_centre_distance = math.inf
        self.collisions = 0
        self.robot_moving_collisions = 0   # contacts while the robot itself was moving
        self.robot_vel = (0.0, 0.0)
        # Smoothness: velocity samples every >= 0.1 s of sim time while running.
        self.last_sample = None           # (t, v, w)
        self.abs_lin_acc, self.abs_ang_acc, self.acc_samples = 0.0, 0.0, 0
        self.stops, self.was_moving, self.stopped_since = 0, False, None
        self.humans_xy = []
        self.avoid_onset_distance = None  # distance to nearest person when first leaving the line
        self.intrusion_time = 0.0
        self.in_collision = set()
        self.running = False
        self.last_human_stamp = None
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_subscription(Agents, 'human_states', self.on_humans, 10)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.running and self.pose is not None:
            self.path_length += math.dist(self.pose, (p.x, p.y))
        self.pose = (p.x, p.y)
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear.x   # body frame (diff drive)
        self.robot_vel = (v * math.cos(yaw), v * math.sin(yaw))
        if self.running:
            self.track_smoothness(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, v,
                                  msg.twist.twist.angular.z)

    def track_smoothness(self, t, v, w):
        if self.last_sample is None or t - self.last_sample[0] >= 0.1:
            if self.last_sample is not None:
                dt = t - self.last_sample[0]
                self.abs_lin_acc += abs(v - self.last_sample[1]) / dt
                self.abs_ang_acc += abs(w - self.last_sample[2]) / dt
                self.acc_samples += 1
            self.last_sample = (t, v, w)
        # A stop = slowing below 0.05 m/s for >= 0.3 s after having moved, then moving on.
        if abs(v) > 0.1:
            if self.stopped_since is not None and t - self.stopped_since >= 0.3:
                self.stops += 1
            self.was_moving, self.stopped_since = True, None
        elif abs(v) < 0.05 and self.was_moving and self.stopped_since is None:
            self.stopped_since = t
        # First time the robot leaves the straight START->GOAL line by > 0.15 m.
        if self.avoid_onset_distance is None and self.humans_xy:
            (sx, sy), (gx, gy) = self.level['robot_start'][:2], self.level['robot_goal'][:2]
            length = math.hypot(gx - sx, gy - sy)
            lateral = abs((gx - sx) * (self.pose[1] - sy) - (gy - sy) * (self.pose[0] - sx)) / max(length, 1e-6)
            if lateral > 0.15:
                self.avoid_onset_distance = min(math.dist(self.pose, h) for h in self.humans_xy)

    def on_humans(self, msg):
        now = time.monotonic()
        dt = 0.0 if self.last_human_stamp is None else now - self.last_human_stamp
        self.last_human_stamp = now
        self.humans_xy = [(a.position.position.x, a.position.position.y) for a in msg.agents]
        if not self.running or self.pose is None:
            return
        intruded = False
        for agent in msg.agents:
            centre = math.dist(self.pose, (agent.position.position.x, agent.position.position.y))
            clearance = centre - agent.radius - self.args.robot_radius
            self.min_centre_distance = min(self.min_centre_distance, centre)
            self.min_clearance = min(self.min_clearance, clearance)
            intruded |= centre < PERSONAL_SPACE
            if clearance <= 0.0 and agent.id not in self.in_collision:
                self.collisions += 1
                # A robot that had stopped (< 0.1 m/s) was walked into; otherwise it took part.
                if math.hypot(*self.robot_vel) >= 0.1:
                    self.robot_moving_collisions += 1
                self.in_collision.add(agent.id)
            elif clearance > 0.05:
                self.in_collision.discard(agent.id)
        if intruded:
            self.intrusion_time += dt

    def goal_pose(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        x, y = self.level['robot_goal'][:2]
        sx, sy = self.level['robot_start'][:2]
        yaw = math.atan2(y - sy, x - sx)
        goal.pose.position.x, goal.pose.position.y = float(x), float(y)
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return goal

    def run(self):
        log = self.get_logger()
        log.info('Waiting for Nav2 (navigate_to_pose)...')
        if not self.nav.wait_for_server(timeout_sec=self.args.nav_timeout):
            log.error('Nav2 did not come up')
            return 1
        deadline = time.monotonic() + self.args.start_delay
        while time.monotonic() < deadline:  # let costmaps fill with the first scans
            rclpy.spin_once(self, timeout_sec=0.1)
        goal = self.goal_pose()
        log.info(f'Sending goal ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})')
        # The action server appears before bt_navigator is activated and rejects goals until then.
        give_up = time.monotonic() + self.args.nav_timeout
        while True:
            send = self.nav.send_goal_async(NavigateToPose.Goal(pose=goal))
            rclpy.spin_until_future_complete(self, send)
            handle = send.result()
            if handle and handle.accepted:
                break
            if time.monotonic() > give_up:
                log.error('Goal rejected: Nav2 never became active')
                return 1
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
        started = self.get_clock().now()
        self.running = True
        result_future = handle.get_result_async()
        timeout = time.monotonic() + self.args.mission_timeout
        while not result_future.done() and time.monotonic() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result_future.done():
            handle.cancel_goal_async()
            status = 'timeout'
        else:
            status = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
                      GoalStatus.STATUS_CANCELED: 'canceled'}.get(result_future.result().status, 'unknown')
        self.running = False
        duration = (self.get_clock().now() - started).nanoseconds / 1e9
        remaining = math.dist(self.pose, self.level['robot_goal'][:2]) if self.pose else None
        summary = {
            'level': self.args.level, 'method': self.args.method, 'people_source': self.args.people_source,
            'status': status, 'success': status == 'succeeded',
            'time_s': round(duration, 2), 'path_length_m': round(self.path_length, 2),
            'distance_to_goal_m': round(remaining, 2) if remaining is not None else None,
            'min_clearance_to_person_m': round(self.min_clearance, 3) if math.isfinite(self.min_clearance) else None,
            'min_centre_distance_m': round(self.min_centre_distance, 3) if math.isfinite(self.min_centre_distance) else None,
            'personal_space_intrusion_s': round(self.intrusion_time, 2),
            'collisions': self.collisions,
            'robot_moving_collisions': self.robot_moving_collisions,
            'stops': self.stops,
            'mean_abs_lin_acc': round(self.abs_lin_acc / max(self.acc_samples, 1), 3),
            'mean_abs_ang_acc': round(self.abs_ang_acc / max(self.acc_samples, 1), 3),
            'avoid_onset_distance_m': (round(self.avoid_onset_distance, 2)
                                       if self.avoid_onset_distance is not None else None),
            'humans_yield_to_robot': self.args.humans_yield,
        }
        out_dir = Path(os.environ.get('SIM_RESULTS_DIR', '.runtime/results'))
        out_dir.mkdir(parents=True, exist_ok=True)
        out = out_dir / f'mission_{self.args.level}_{self.args.method}_{datetime.now():%Y%m%d_%H%M%S}.json'
        out.write_text(json.dumps(summary, indent=2) + '\n')
        log.info(f'Mission {status}: {json.dumps(summary)} -> {out}')
        return 0 if summary['success'] else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--level', required=True)
    parser.add_argument('--robot-radius', type=float, default=0.175)
    parser.add_argument('--start-delay', type=float, default=3.0, help='Wall seconds before sending the goal')
    parser.add_argument('--nav-timeout', type=float, default=120.0)
    parser.add_argument('--mission-timeout', type=float, default=300.0)
    parser.add_argument('--humans-yield', default='false')
    parser.add_argument('--method', default='unknown')
    parser.add_argument('--people-source', default='ground_truth')
    args, ros_args = parser.parse_known_args()
    with open(args.scenario) as handle:
        level = yaml.safe_load(handle)['level']['ros__parameters']
    rclpy.init(args=ros_args)
    node = Mission(args, level)
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
