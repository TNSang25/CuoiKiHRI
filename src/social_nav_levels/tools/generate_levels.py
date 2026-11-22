#!/usr/bin/env python3
"""Generate the five social-navigation levels from one geometric description.

For every level this writes, from the same boxes so they always agree:
  worlds/levelN.world                 Gazebo Classic world (one static model per box)
  maps/levelN.pgm + maps/levelN.yaml  2-D occupancy map for map_server / RViz
  scenarios/agents_levelN.yaml        people + robot start/goal (``level`` block)
  behavior_trees/agents_levelN__agent_K_bt.xml

Usage:  python3 src/social_nav_levels/tools/generate_levels.py [--only 3]
"""
import argparse
import math
import random
from pathlib import Path

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
RESOLUTION = 0.05
WALL_T = 0.15
WALL_H = 2.0
WALL_COLOR = '0.82 0.80 0.76 1'
WOOD = '0.55 0.36 0.20 1'
DARK = '0.25 0.25 0.28 1'
HUMAN_RADIUS = 0.3
HUMAN_SPEED_SCALE = 0.8   # every person's desired speed is multiplied by this
LEG_CLEARANCE = 0.3    # every straight leg of a route must keep a body radius from obstacles
DOOR_LANE = 0.25       # level 5 keep-right lane offset inside the 1.2 m door
GOAL_CLEARANCE = 0.55  # people stall short of goals tucked into corners


# ----------------------------------------------------------------------------- geometry helpers
class Level:
    def __init__(self, name, title, camera):
        self.name, self.title, self.camera = name, title, camera
        self.boxes = []        # (name, cx, cy, sx, sy, h, color, kind)
        self.cylinders = []    # (name, cx, cy, r, h, color)
        self.free = []         # rectangles (x0, y0, x1, y1) that are walkable floor
        self.agents = []
        self.robot_start = None
        self.robot_goal = None
        self.waypoints = set()  # pass-through points allowed in narrow gaps (doorways)

    def wall(self, x0, y0, x1, y1):
        """Axis-aligned wall segment along its centre line."""
        n = len(self.boxes)
        if abs(y0 - y1) < 1e-9:
            self.boxes.append((f'wall_{n}', (x0 + x1) / 2, y0, abs(x1 - x0) + WALL_T, WALL_T,
                               WALL_H, WALL_COLOR, 'wall'))
        elif abs(x0 - x1) < 1e-9:
            self.boxes.append((f'wall_{n}', x0, (y0 + y1) / 2, WALL_T, abs(y1 - y0) + WALL_T,
                               WALL_H, WALL_COLOR, 'wall'))
        else:
            raise ValueError('walls must be axis aligned')

    def room(self, x0, y0, x1, y1):
        self.free.append((x0, y0, x1, y1))

    def table(self, cx, cy, sx, sy, chairs=True):
        n = len(self.boxes)
        self.boxes.append((f'table_{n}', cx, cy, sx, sy, 0.75, WOOD, 'table'))
        if chairs:
            along_x = sx >= sy
            count = max(1, int((sx if along_x else sy) // 0.7))
            for side in (-1, 1):
                for k in range(count):
                    t = (k + 0.5) / count - 0.5
                    if along_x:
                        x, y = cx + t * sx, cy + side * (sy / 2 + 0.3)
                    else:
                        x, y = cx + side * (sx / 2 + 0.3), cy + t * sy
                    self.boxes.append((f'chair_{len(self.boxes)}', x, y, 0.42, 0.42, 0.9, DARK, 'chair'))

    def block(self, name, cx, cy, sx, sy, h, color=WOOD):
        self.boxes.append((f'{name}_{len(self.boxes)}', cx, cy, sx, sy, h, color, 'block'))

    def pillar(self, cx, cy, r=0.25):
        self.cylinders.append((f'pillar_{len(self.cylinders)}', cx, cy, r, WALL_H, WALL_COLOR))

    def person(self, start, heading, goals, *, vel=1.1, skin=None, group=-1, behavior='Regular',
               tree='regular', goal_radius=0.3, release_distance=4.5, pause=0.0):
        aid = len(self.agents) + 1
        self.agents.append(dict(id=aid, start=start, heading=heading, goals=goals, vel=vel,
                                skin=(aid - 1) % 5 if skin is None else skin, group=group,
                                behavior=behavior, tree=tree, goal_radius=goal_radius, pause=pause,
                                release_distance=release_distance))

    def shift_robot_to_origin(self):
        """Translate the whole level so the robot starts at (0, 0) facing +x.

        Gazebo world, odom and map frames then coincide at the robot's spawn pose.
        """
        dx, dy = -self.robot_start[0], -self.robot_start[1]
        if abs(self.robot_start[2]) > 1e-9:
            raise ValueError(f'{self.name}: robot start yaw must be 0')

        def move(p):
            return (round(p[0] + dx, 3), round(p[1] + dy, 3))
        self.boxes = [(n, cx + dx, cy + dy, *rest) for n, cx, cy, *rest in self.boxes]
        self.cylinders = [(n, cx + dx, cy + dy, *rest) for n, cx, cy, *rest in self.cylinders]
        self.free = [(x0 + dx, y0 + dy, x1 + dx, y1 + dy) for x0, y0, x1, y1 in self.free]
        for a in self.agents:
            a['start'] = move(a['start'])
            a['goals'] = [move(g) for g in a['goals']]
        self.waypoints = {move(w) for w in self.waypoints}
        self.robot_start = (0.0, 0.0, 0.0)
        self.robot_goal = move(self.robot_goal)
        cam = list(self.camera)
        cam[0] += dx
        cam[1] += dy
        self.camera = tuple(cam)

    def bounds(self):
        xs, ys = [], []
        for _, cx, cy, sx, sy, *_ in self.boxes:
            xs += [cx - sx / 2, cx + sx / 2]
            ys += [cy - sy / 2, cy + sy / 2]
        return min(xs) - 1.0, min(ys) - 1.0, max(xs) + 1.0, max(ys) + 1.0


# ----------------------------------------------------------------------------- level definitions
def level1():
    lv = Level('level1', 'Level 1 - empty 6x6 m room, one crossing pedestrian', (0, -7.5, 9, 0, 0.9, 1.5708))
    h = 3.0
    lv.room(-h, -h, h, h)
    for a, b in [((-h, -h), (h, -h)), ((-h, h), (h, h)), ((-h, -h), (-h, h)), ((h, -h), (h, h))]:
        lv.wall(*a, *b)
    lv.robot_start, lv.robot_goal = (-2.3, 0.0, 0.0), (2.3, 0.0)
    # Walks back and forth perpendicular to the robot's straight line.
    # Easiest level: a slower walker who also stands for 3 s at each end, leaving a clear gap.
    lv.person((0.3, -2.3), math.pi / 2, [(0.3, 2.3), (-0.3, -2.3)], vel=0.8, pause=3.0)
    return lv


def level2():
    lv = Level('level2', 'Level 2 - 2.5 m wide corridor, one oncoming pedestrian', (0, -8, 10, 0, 0.95, 1.5708))
    L, w = 7.0, 1.25
    lv.room(-L, -w, L, w)
    lv.wall(-L, -w, L, -w)
    lv.wall(-L, w, L, w)
    lv.wall(-L, -w, -L, w)
    lv.wall(L, -w, L, w)
    lv.robot_start, lv.robot_goal = (-6.0, 0.0, 0.0), (6.0, 0.0)
    # Starts at the far end and walks towards the robot, then turns back.
    lv.person((6.0, 0.35), math.pi, [(-6.2, 0.35), (6.2, -0.35)], vel=1.1)
    return lv


def level3():
    lv = Level('level3', 'Level 3 - L-shaped corridor with a blind corner', (-3, -9, 13, 0, 1.0, 1.5708))
    lv.room(-8, -1, 1, 1)      # leg A (robot starts here, along x)
    lv.room(-1, -1, 1, 8)      # leg B (goal, along y)
    lv.wall(-8, -1, 1, -1)     # outer bottom
    lv.wall(1, -1, 1, 8)       # outer right
    lv.wall(-8, 1, -1, 1)      # inner top of leg A
    lv.wall(-1, 1, -1, 8)      # inner left of leg B -> hides leg B from leg A
    lv.wall(-8, -1, -8, 1)     # end of leg A
    lv.wall(-1, 8, 1, 8)       # end of leg B
    lv.robot_start, lv.robot_goal = (-7.0, 0.0, 0.0), (0.0, 7.0)
    corner = (0.1, 0.0)
    # Oncoming person: visible at the corner, walks down leg A towards the robot.
    lv.person((-0.2, 0.3), math.pi, [(-7.2, 0.35), corner, (0.3, 7.2), corner], vel=1.1)
    # Hidden behind the corner in leg B, waits until the robot gets close, then comes out.
    lv.person((0.3, 3.2), -math.pi / 2, [corner, (-7.2, -0.35), corner, (-0.3, 7.2)], vel=1.2,
              tree='hidden', release_distance=4.5)
    return lv


def level4():
    lv = Level('level4', 'Level 4 - furnished room, standing group of 3 + 2 walkers', (0, -10, 11, 0, 0.95, 1.5708))
    h = 4.0
    lv.room(-h, -h, h, h)
    for a, b in [((-h, -h), (h, -h)), ((-h, h), (h, h)), ((-h, -h), (-h, h)), ((h, -h), (h, h))]:
        lv.wall(*a, *b)
    lv.table(-2.0, 2.1, 1.4, 0.8)
    lv.table(2.2, -2.1, 1.4, 0.8)
    lv.block('cabinet', 1.2, 3.7, 1.6, 0.45, 1.8)
    lv.block('sofa', -3.55, -1.6, 0.7, 1.6, 0.8, DARK)
    lv.robot_start, lv.robot_goal = (-3.2, -3.2, 0.0), (3.2, 3.2)
    # Group of three talking in a circle right on the robot's diagonal.
    gx, gy, gr = 0.7, 0.7, 0.55
    for k in range(3):
        ang = math.pi / 2 + k * 2 * math.pi / 3
        px, py = gx + gr * math.cos(ang), gy + gr * math.sin(ang)
        lv.person((px, py), ang + math.pi, [(px, py)], vel=0.8, group=1, goal_radius=0.4)
    lv.person((-3.2, -0.1), 0.0, [(3.2, 0.2), (-3.2, -0.1)], vel=1.0)
    lv.person((-0.7, 3.2), -math.pi / 2, [(-0.5, -3.2), (-0.7, 3.2)], vel=1.2)
    return lv


def level5():
    lv = Level('level5', 'Level 5 - lobby + office joined by a 1.2 m door, 6 pedestrians',
               (0, -12, 15, 0, 0.95, 1.5708))
    W, H, door = 6.0, 4.0, 0.6
    lv.room(-W, -H, W, H)
    lv.wall(-W, -H, W, -H)
    lv.wall(-W, H, W, H)
    lv.wall(-W, -H, -W, H)
    lv.wall(W, -H, W, H)
    lv.wall(0, -H, 0, -door - WALL_T / 2)
    lv.wall(0, door + WALL_T / 2, 0, H)
    lv.pillar(-3.0, 1.6)
    lv.pillar(-3.0, -1.6)
    lv.block('reception', -5.3, 2.6, 0.7, 2.0, 1.1)
    lv.table(2.8, 2.6, 1.6, 0.7)
    lv.table(2.8, -2.6, 1.6, 0.7)
    lv.block('desk', 5.3, 0.0, 0.7, 1.6, 0.75)
    # Goal in the open aisle between the two office tables (the desk corner made MPPI dither).
    lv.robot_start, lv.robot_goal = (-5.0, -2.8, 0.0), (3.2, 1.0)
    # Random destinations, kept >1 m away from the robot's start and goal.
    # Destinations keep clear of the robot's lane (lobby corner -> door -> office aisle): none
    # right in front of the door, and office ones only along the walls and in the corners.
    lobby = [(-3.6, -3.2), (-4.6, 0.0), (-3.0, 3.2), (-1.6, 2.4), (-1.8, -3.1),
             (-4.0, -1.0), (-2.2, 0.9)]
    office = [(1.0, 2.6), (1.0, -2.4), (2.8, -1.0), (4.4, -1.4), (4.5, 2.9), (4.5, -2.9)]
    grid = OccupancyGrid(lv)
    rng = random.Random(7)

    def door(from_room):
        # Keep right through the door: lobby->office walks at y<0, office->lobby at y>0.
        side, lane = (-1, -DOOR_LANE) if from_room == 'lobby' else (1, DOOR_LANE)
        return [(side * 1.3, lane), (-side * 1.3, lane)]

    def wander(prev, room, count):
        points = []
        for _ in range(count):
            choices = [p for p in (lobby if room == 'lobby' else office) if p != prev and grid.leg_ok(prev, p)]
            prev = rng.choice(choices)
            points.append(prev)
        return points

    def route(start, room, crosses):
        """Cyclic route; crossers visit the other room once per loop (there and back)."""
        other = 'office' if room == 'lobby' else 'lobby'
        if not crosses:
            return wander(start, room, 4) + [start]
        points = wander(start, room, 1) + door(room)
        points += wander(points[-1], other, 2) + door(other)
        return points + wander(points[-1], room, 1) + [start]

    # (x, y, room, crosses the door) - three of the eight people use the door, the rest stay in
    # their room, so the doorway sees regular two-way traffic without permanent jams.
    # Six people (the table's minimum): one walks back and forth through the door, the other five
    # wander inside their room and stand for 2 s at each destination, so the door is usually free
    # and the robot can get through without long waits. Nobody starts next to the robot's start
    # or goal.
    starts = [(-3.8, 2.4, 'lobby', False), (-1.0, -2.2, 'lobby', False), (-1.2, 1.2, 'lobby', True),
              (1.8, 1.0, 'office', False), (3.8, -0.4, 'office', False), (1.6, -1.4, 'office', False)]
    for x, y, room, crosses in starts:
        for _ in range(200):
            pts = route((x, y), room, crosses)
            if grid.route_ok((x, y), pts):
                break
        else:
            raise SystemExit(f'level5: no walkable route found from ({x}, {y})')
        first = pts[0]
        # The door-crosser does not pause: its route has waypoints right in front of the door.
        lv.person((x, y), math.atan2(first[1] - y, first[0] - x), pts, vel=round(rng.uniform(0.85, 1.2), 2),
                  goal_radius=0.5, pause=0.0 if crosses else 2.0)
    return lv


class OccupancyGrid:
    """Rasterised level: 254 free, 0 occupied, 205 unknown (outside the rooms)."""

    def __init__(self, lv):
        x0, y0, x1, y1 = lv.bounds()
        w, h = int(math.ceil((x1 - x0) / RESOLUTION)), int(math.ceil((y1 - y0) / RESOLUTION))
        self.origin = (x0, y1 - h * RESOLUTION)
        cells = np.full((h, w), 205, dtype=np.uint8)
        X, Y = np.meshgrid(x0 + (np.arange(w) + 0.5) * RESOLUTION, y1 - (np.arange(h) + 0.5) * RESOLUTION)
        for fx0, fy0, fx1, fy1 in lv.free:
            cells[(X >= fx0) & (X <= fx1) & (Y >= fy0) & (Y <= fy1)] = 254
        for _, cx, cy, sx, sy, *_ in lv.boxes:
            cells[(np.abs(X - cx) <= sx / 2) & (np.abs(Y - cy) <= sy / 2)] = 0
        for _, cx, cy, r, *_ in lv.cylinders:
            cells[np.hypot(X - cx, Y - cy) <= r] = 0
        self.cells = cells
        rr, cc = np.nonzero(cells != 254)
        self.blocked = np.column_stack([X[rr, cc], Y[rr, cc]])

    def clearance(self, x, y, cap=1.0):
        near = self.blocked[(np.abs(self.blocked[:, 0] - x) <= cap) & (np.abs(self.blocked[:, 1] - y) <= cap)]
        if near.size == 0:
            return cap
        return float(min(cap, np.min(np.hypot(near[:, 0] - x, near[:, 1] - y))))

    def leg_ok(self, a, b):
        steps = max(1, int(math.dist(a, b) / 0.1))
        return all(self.clearance(a[0] + (b[0] - a[0]) * t / steps, a[1] + (b[1] - a[1]) * t / steps)
                   >= LEG_CLEARANCE for t in range(steps + 1))

    def route_ok(self, start, goals):
        points = [start, *goals, goals[0]]
        return all(self.leg_ok(p, q) for p, q in zip(points, points[1:]))


LEVELS = {1: level1, 2: level2, 3: level3, 4: level4, 5: level5}


# ----------------------------------------------------------------------------- writers
def box_model(name, cx, cy, sx, sy, h, color, kind):
    collision = (f'<collision name="collision"><pose>0 0 {h / 2:.3f} 0 0 0</pose>'
                 f'<geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry></collision>')
    mat = f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>'
    if kind == 'table':
        top = 0.04
        visuals = [f'<visual name="top"><pose>0 0 {h - top / 2:.3f} 0 0 0</pose><geometry><box>'
                   f'<size>{sx:.3f} {sy:.3f} {top}</size></box></geometry>{mat}</visual>']
        for i, (lx, ly) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
            visuals.append(f'<visual name="leg{i}"><pose>{lx * (sx / 2 - 0.06):.3f} {ly * (sy / 2 - 0.06):.3f} '
                           f'{(h - top) / 2:.3f} 0 0 0</pose><geometry><box><size>0.05 0.05 {h - top:.3f}</size>'
                           f'</box></geometry>{mat}</visual>')
        visual = ''.join(visuals)
    elif kind == 'chair':
        visual = (f'<visual name="seat"><pose>0 0 0.225 0 0 0</pose><geometry><box><size>{sx} {sy} 0.45</size>'
                  f'</box></geometry>{mat}</visual>')
        collision = (f'<collision name="collision"><pose>0 0 0.225 0 0 0</pose><geometry><box>'
                     f'<size>{sx} {sy} 0.45</size></box></geometry></collision>')
    else:
        visual = (f'<visual name="visual"><pose>0 0 {h / 2:.3f} 0 0 0</pose><geometry><box>'
                  f'<size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry>{mat}</visual>')
    return (f'<model name="{name}"><static>true</static><pose>{cx:.3f} {cy:.3f} 0 0 0 0</pose>'
            f'<link name="link">{collision}{visual}</link></model>')


def cylinder_model(name, cx, cy, r, h, color):
    geo = f'<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>'
    return (f'<model name="{name}"><static>true</static><pose>{cx:.3f} {cy:.3f} {h / 2:.3f} 0 0 0</pose>'
            f'<link name="link"><collision name="collision">{geo}</collision><visual name="visual">{geo}'
            f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>')


def write_world(lv, path):
    cam = ' '.join(f'{v:g}' for v in lv.camera)
    models = '\n    '.join([box_model(*b) for b in lv.boxes] + [cylinder_model(*c) for c in lv.cylinders])
    path.write_text(f'''<?xml version="1.0" encoding="utf-8"?>
<!-- {lv.title}. Generated by tools/generate_levels.py - edit that file instead. -->
<sdf version="1.6">
  <world name="default">
    <gui><camera name="gzclient_camera"><pose>{cam}</pose></camera></gui>
    <gravity>0 0 -9.8</gravity>
    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.5 0.5 0.5 1</ambient><background>0.7 0.75 0.8 1</background>
      <shadows>false</shadows><grid>false</grid><origin_visual>false</origin_visual>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>
      <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        <material><ambient>0.62 0.62 0.6 1</ambient><diffuse>0.62 0.62 0.6 1</diffuse></material></visual>
    </link></model>
    {models}
  </world>
</sdf>
''')


def write_map(lv, pgm, yml):
    grid = OccupancyGrid(lv)
    h, w = grid.cells.shape
    pgm.write_bytes(f'P5\n# {lv.name}\n{w} {h}\n255\n'.encode() + grid.cells.tobytes())
    yml.write_text(f'image: {pgm.name}\nmode: trinary\nresolution: {RESOLUTION}\n'
                   f'origin: [{grid.origin[0]:.3f}, {grid.origin[1]:.3f}, 0]\n'
                   'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.19\n')
    return grid


def check_layout(lv, grid):
    """Fail loudly if a pose is too close to an obstacle or a person's route crosses one.

    The people have no global planner (pure social force), so every straight leg of a
    route, including the wrap-around back to the first goal, must be walkable.
    """
    problems = []
    for x, y, *_ in [lv.robot_start, lv.robot_goal]:
        if grid.clearance(x, y) < 0.4:
            problems.append(f'robot pose ({x}, {y})')
    for a in lv.agents:
        for x, y in [a['start'], *a['goals']]:
            if (x, y) not in lv.waypoints and grid.clearance(x, y) < GOAL_CLEARANCE:
                problems.append(f'agent {a["id"]} point ({x:.2f}, {y:.2f}) clearance {grid.clearance(x, y):.2f}')
        if not grid.route_ok(a['start'], a['goals']):
            problems.append(f'agent {a["id"]} route crosses an obstacle')
    if problems:
        raise SystemExit(f'{lv.name}: layout problems:\n  ' + '\n  '.join(problems))


def write_scenario(lv, path):
    goals, agents = {}, {}
    gid = {}

    def goal_id(p):
        key = (round(p[0], 3), round(p[1], 3))
        if key not in gid:
            gid[key] = len(gid) + 1
            goals[gid[key]] = {'x': key[0], 'y': key[1]}
        return gid[key]

    names = []
    for a in lv.agents:
        name = f'agent{a["id"]}'
        names.append(name)
        agents[name] = {
            'id': a['id'], 'group_id': a['group'], 'skin': a['skin'],
            'max_vel': round(a['vel'] * HUMAN_SPEED_SCALE, 2),
            'radius': HUMAN_RADIUS, 'goal_radius': a['goal_radius'], 'cyclic_goals': True,
            'init_pose': {'x': round(a['start'][0], 3), 'y': round(a['start'][1], 3), 'z': 1.25,
                          'h': round(a['heading'], 3)},
            'behavior': {'type': a['behavior'], 'configuration': 1, 'goal_force_factor': 2.0,
                         'obstacle_force_factor': 10.0, 'social_force_factor': 5.0,
                         'other_force_factor': 20.0},
            'goals': [goal_id(p) for p in a['goals']],
        }
    params = {'yaml_base_name': f'agents_{lv.name}', 'simulator': 'Gazebo Classic', 'map': lv.name,
              # /people comes from social_nav_bringup's people_adapter (one source for the nav stack).
              'publish_people': False, 'global_goals': goals, 'agents': names, **agents}
    level = {'title': lv.title,
             'robot_start': [round(v, 3) for v in lv.robot_start],
             'robot_goal': [round(v, 3) for v in lv.robot_goal]}
    text = (f'# {lv.title}\n# Generated by tools/generate_levels.py - edit that file instead.\n' +
            yaml.safe_dump({'agent_loader': {'ros__parameters': params},
                            'level': {'ros__parameters': level}}, sort_keys=False))
    path.write_text(text)


def regular_nav(indent):
    lines = ['<Fallback>',
             '  <Sequence>',
             '    <Inverter><IsGoalReached agent_id="{id}"/></Inverter>',
             '    <RegularNav agent_id="{id}" time_step="{dt}"/>',
             '  </Sequence>',
             '  <UpdateGoal agent_id="{id}"/>',
             '</Fallback>']
    return '\n'.join(' ' * indent + line for line in lines)


def write_tree(lv, a, path):
    tid = f'{lv.name}_agent{a["id"]}'
    if a['tree'] == 'hidden':
        body = f'''    <Sequence>
      <RunOnce then_skip="true"><Script code="released:=false"/></RunOnce>
      <Fallback>
        <!-- Stay still behind the corner until the robot is within {a['release_distance']} m. -->
        <Sequence>
          <ScriptCondition code="released==false"/>
          <Inverter><IsRobotClose agent_id="{{id}}" threshold="{a['release_distance']}"/></Inverter>
        </Sequence>
        <Sequence>
          <Script code="released:=true"/>
{regular_nav(10)}
        </Sequence>
      </Fallback>
    </Sequence>'''
    elif a['pause'] > 0.0:
        # Walk to the goal, stand still for `pause` seconds, then head for the next goal.
        body = f'''    <Fallback>
      <Sequence>
        <Inverter><IsGoalReached agent_id="{{id}}"/></Inverter>
        <RegularNav agent_id="{{id}}" time_step="{{dt}}"/>
      </Sequence>
      <Sequence>
        <StopAndWaitTimerAction agent_id="{{id}}" wait_duration="{a['pause']}"/>
        <UpdateGoal agent_id="{{id}}"/>
      </Sequence>
    </Fallback>'''
    else:
        body = regular_nav(4)
    path.write_text(f'''<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by tools/generate_levels.py - edit that file instead. -->
<root BTCPP_format="4" main_tree_to_execute="{tid}">
  <BehaviorTree ID="{tid}">
{body}
  </BehaviorTree>
</root>
''')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--only', type=int, choices=sorted(LEVELS))
    args = parser.parse_args()
    for number, build in LEVELS.items():
        if args.only and number != args.only:
            continue
        lv = build()
        lv.shift_robot_to_origin()
        write_world(lv, PACKAGE / 'worlds' / f'{lv.name}.world')
        grid = write_map(lv, PACKAGE / 'maps' / f'{lv.name}.pgm', PACKAGE / 'maps' / f'{lv.name}.yaml')
        check_layout(lv, grid)
        write_scenario(lv, PACKAGE / 'scenarios' / f'agents_{lv.name}.yaml')
        for old in (PACKAGE / 'behavior_trees').glob(f'agents_{lv.name}__agent_*_bt.xml'):
            old.unlink()
        for a in lv.agents:
            write_tree(lv, a, PACKAGE / 'behavior_trees' / f'agents_{lv.name}__agent_{a["id"]}_bt.xml')
        print(f'{lv.name}: {len(lv.boxes) + len(lv.cylinders)} obstacles, {len(lv.agents)} people')


if __name__ == '__main__':
    main()
