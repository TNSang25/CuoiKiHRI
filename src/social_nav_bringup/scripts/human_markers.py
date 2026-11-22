#!/usr/bin/env python3
"""Publish RViz markers for the simulated people, the robot and the level's robot start/goal.

Topics:
  /bot_markers            visualization_msgs/MarkerArray  people, personal space, heading, robot, goal
  /level/robot_start      geometry_msgs/PoseStamped (latched)
  /level/robot_goal       geometry_msgs/PoseStamped (latched)
"""
import argparse
import math

import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from social_nav_msgs.msg import Agent, Agents

PALETTE = [(0.90, 0.30, 0.24), (0.95, 0.61, 0.07), (0.56, 0.27, 0.68), (0.10, 0.74, 0.61),
           (0.91, 0.12, 0.55), (0.55, 0.55, 0.10), (0.20, 0.60, 0.86), (0.36, 0.72, 0.36)]
PERSONAL_SPACE = 1.2   # Hall's personal-distance boundary [m]
FRAME = 'map'


def quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class HumanMarkers(Node):
    def __init__(self, level, robot_radius):
        super().__init__('human_markers')
        self.level = level
        self.robot_radius = robot_radius
        self.robot = None
        self.pub = self.create_publisher(MarkerArray, 'human_markers', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.start_pub = self.create_publisher(PoseStamped, 'level/robot_start', latched)
        self.goal_pub = self.create_publisher(PoseStamped, 'level/robot_goal', latched)
        self.create_subscription(Agents, 'human_states', self.on_humans, 10)
        self.create_subscription(Agent, 'robot_states', self.on_robot, 10)
        self.publish_level_poses()

    def publish_level_poses(self):
        for key, pub in [('robot_start', self.start_pub), ('robot_goal', self.goal_pub)]:
            values = self.level.get(key)
            if not values:
                continue
            msg = PoseStamped()
            msg.header.frame_id = FRAME
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y = float(values[0]), float(values[1])
            yaw = float(values[2]) if len(values) > 2 else 0.0
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = quaternion(yaw)
            pub.publish(msg)

    def on_robot(self, msg):
        self.robot = msg

    def marker(self, ns, mid, kind, stamp):
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = stamp
        m.ns, m.id, m.type, m.action = ns, mid, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime.nanosec = 500_000_000
        return m

    def on_humans(self, msg):
        stamp = msg.header.stamp
        out = MarkerArray()
        for agent in msg.agents:
            r, g, b = PALETTE[(agent.id - 1) % len(PALETTE)]
            x, y = agent.position.position.x, agent.position.position.y
            speed = math.hypot(agent.velocity.linear.x, agent.velocity.linear.y)

            body = self.marker('people', agent.id, Marker.CYLINDER, stamp)
            body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, 0.85
            body.scale.x = body.scale.y = 2 * agent.radius
            body.scale.z = 1.7
            body.color.r, body.color.g, body.color.b, body.color.a = r, g, b, 0.9
            out.markers.append(body)

            space = self.marker('personal_space', agent.id, Marker.LINE_STRIP, stamp)
            space.scale.x = 0.03
            space.color.r, space.color.g, space.color.b, space.color.a = r, g, b, 0.6
            for k in range(41):
                a = 2 * math.pi * k / 40
                space.points.append(Point(x=x + PERSONAL_SPACE * math.cos(a),
                                          y=y + PERSONAL_SPACE * math.sin(a), z=0.02))
            out.markers.append(space)

            heading = self.marker('heading', agent.id, Marker.ARROW, stamp)
            length = 0.4 + 0.6 * min(speed, 1.5)
            heading.points = [Point(x=x, y=y, z=1.75),
                              Point(x=x + length * math.cos(agent.yaw),
                                    y=y + length * math.sin(agent.yaw), z=1.75)]
            heading.scale.x, heading.scale.y, heading.scale.z = 0.06, 0.14, 0.14
            heading.color.r = heading.color.g = heading.color.b = 0.1
            heading.color.a = 1.0
            out.markers.append(heading)

            label = self.marker('labels', agent.id, Marker.TEXT_VIEW_FACING, stamp)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 2.1
            label.scale.z = 0.35
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'{agent.id}  {speed:.1f} m/s'
            out.markers.append(label)

        if self.robot is not None:
            p = self.robot.position.position
            # Flat footprint disc; the TurtleBot 4 mesh itself comes from the RobotModel display.
            robot = self.marker('robot', 0, Marker.CYLINDER, stamp)
            robot.pose.position.x, robot.pose.position.y, robot.pose.position.z = p.x, p.y, 0.015
            robot.scale.x = robot.scale.y = 2 * self.robot_radius
            robot.scale.z = 0.03
            robot.color.r, robot.color.g, robot.color.b, robot.color.a = 0.05, 0.3, 0.9, 0.6
            out.markers.append(robot)

        for idx, (key, color, text) in enumerate([('robot_start', (0.1, 0.7, 0.1), 'START'),
                                                  ('robot_goal', (0.1, 0.1, 0.1), 'GOAL')]):
            values = self.level.get(key)
            if not values:
                continue
            disc = self.marker('robot_task', idx, Marker.CYLINDER, stamp)
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = values[0], values[1], 0.01
            disc.scale.x = disc.scale.y = 0.7
            disc.scale.z = 0.02
            disc.color.r, disc.color.g, disc.color.b, disc.color.a = *color, 0.6
            disc.lifetime.nanosec = 0
            out.markers.append(disc)
            tag = self.marker('robot_task', 10 + idx, Marker.TEXT_VIEW_FACING, stamp)
            tag.pose.position.x, tag.pose.position.y, tag.pose.position.z = values[0], values[1], 0.5
            tag.scale.z = 0.35
            tag.color.r, tag.color.g, tag.color.b, tag.color.a = *color, 1.0
            tag.text = text
            tag.lifetime.nanosec = 0
            out.markers.append(tag)
        self.pub.publish(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', required=True, help='Scenario YAML with an optional level block')
    parser.add_argument('--robot-radius', type=float, default=0.175)
    args, ros_args = parser.parse_known_args()
    with open(args.scenario) as handle:
        level = (yaml.safe_load(handle).get('level') or {}).get('ros__parameters', {})
    rclpy.init(args=ros_args)
    node = HumanMarkers(level, args.robot_radius)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
