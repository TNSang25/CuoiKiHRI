"""Launch a social-navigation level: people, TurtleBot 4, Nav2, people perception and RViz."""
from pathlib import Path
import os
import re
import subprocess
import sys
import uuid
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_RADIUS = 0.175  # TurtleBot 4 (Create 3 base) footprint radius


def turtlebot4_model(urdf, x, y, yaw, run_directory):
    """Convert the TurtleBot 4 URDF to an SDF <model name="robot"> placed at the level start."""
    def to_file(match):
        return f'file://{get_package_share_directory(match.group(1))}/'
    gazebo_urdf = run_directory / 'turtlebot4.urdf'
    gazebo_urdf.write_text(re.sub(r'package://([A-Za-z0-9_]+)/', to_file, urdf))
    sdf = subprocess.run(['gz', 'sdf', '-p', str(gazebo_urdf)], check=True,
                         capture_output=True, text=True).stdout
    model = ET.fromstring(sdf).find('model')
    model.set('name', 'robot')
    pose = model.find('pose') if model.find('pose') is not None else ET.SubElement(model, 'pose')
    pose.text = f'{x} {y} 0.0 0 0 {yaw}'
    return model


def launch_simulation(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    assets = Path(get_package_share_directory('social_nav_levels'))
    bringup = Path(get_package_share_directory('social_nav_bringup'))
    description = Path(get_package_share_directory('tb4_classic_description'))
    world_name = arg('world')
    scenario = Path(arg('scenario')) if arg('scenario') else assets / 'scenarios' / f'agents_{world_name}.yaml'
    scenario_data = yaml.safe_load(scenario.read_text())
    configuration = scenario_data['agent_loader']['ros__parameters']
    level = (scenario_data.get('level') or {}).get('ros__parameters', {})
    robot_x, robot_y, robot_yaw = (list(level.get('robot_start', [])) + [0.0, 0.0, 0.0])[:3]
    agents = configuration['agents']
    if not agents:
        raise ValueError('The scenario must contain at least one agent')
    ids = [configuration[name]['id'] for name in agents]
    if len(ids) != len(set(ids)):
        raise ValueError('Agent IDs must be unique')
    tree_directory = Path(arg('behavior_tree_directory')) if arg('behavior_tree_directory') else assets / 'behavior_trees'
    base_name = configuration['yaml_base_name']
    for agent_id in ids:
        tree_file = tree_directory / f'{base_name}__agent_{agent_id}_bt.xml'
        if not tree_file.is_file():
            raise FileNotFoundError(f'Missing behavior tree: {tree_file}')
    humans_yield = arg('humans_yield').lower() in ('true', '1', 'yes')

    # ---------------------------------------------------------------- world with the robot inside
    runtime = Path(arg('runtime_dir')).resolve()
    run_directory = runtime / 'worlds' / uuid.uuid4().hex[:8]
    run_directory.mkdir(parents=True, exist_ok=True)
    generated = run_directory / 'generatedWorld.world'
    world_tree = ET.parse(assets / 'worlds' / f'{world_name}.world')
    world = world_tree.getroot().find('world')
    world.find('physics/max_step_size').text = '0.005'
    world.find('physics/real_time_update_rate').text = '200'
    robot_urdf = (description / 'urdf' / 'turtlebot4_classic.urdf').read_text()
    world.append(turtlebot4_model(robot_urdf, robot_x, robot_y, robot_yaw, run_directory))
    base_world = run_directory / f'{world_name}.world'
    world_tree.write(base_world, encoding='utf-8', xml_declaration=True)

    # ---------------------------------------------------------------- people
    # People sense static obstacles through Gazebo models; ignore the floor, each other's
    # collision bodies (they already avoid each other socially) and, unless they yield,
    # the robot: then only the robot is responsible for avoiding them.
    ignored = ['ground_plane'] + [f'{name}_body' for name in agents] + ([] if humans_yield else ['robot'])
    loader = Node(package='human_agents', executable='agent_loader', name='agent_loader',
                  output='screen', parameters=[str(scenario)])
    generator = Node(
        package='human_gazebo', executable='world_generator', output='screen',
        parameters=[{
            'base_world': str(base_world), 'robot_name': 'robot', 'robot_radius': ROBOT_RADIUS,
            'use_gazebo_obs': True, 'use_collision': True, 'update_rate': 20.0,
            'global_frame_to_publish': 'map', 'use_navgoal_to_start': False,
            'ignore_models': ' '.join(ignored),
        }])
    controller = Node(
        package='human_agents', executable='agent_controller', output='screen',
        parameters=[{'use_sim_time': True, 'behavior_tree_directory': str(tree_directory),
                     'stuck_goal_timeout': float(arg('stuck_goal_timeout')),
                     'humans_yield_to_robot': humans_yield}])
    server = ExecuteProcess(
        cmd=['gzserver', '--verbose', str(generated),
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    client = ExecuteProcess(cmd=['gzclient'], output='screen',
                            condition=IfCondition(LaunchConfiguration('gui')))
    wait_script = Path(get_package_prefix('social_nav_bringup')) / 'lib/social_nav_bringup/wait_world'
    ready = ExecuteProcess(cmd=[sys.executable, str(wait_script), str(generated), str(len(agents))],
                           output='screen')

    # ---------------------------------------------------------------- robot + navigation
    robot_state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                       output='log', parameters=[{'robot_description': robot_urdf, 'use_sim_time': True}])
    # Ground-truth odometry is expressed in the Gazebo world frame, which is the map frame.
    map_to_odom = Node(package='tf2_ros', executable='static_transform_publisher', output='log',
                       arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
                       parameters=[{'use_sim_time': True}])
    map_file = assets / 'maps' / f'{world_name}.yaml'
    map_server = Node(package='nav2_map_server', executable='map_server', name='map_server',
                      output='screen', parameters=[{'yaml_filename': str(map_file), 'use_sim_time': True}])
    map_lifecycle = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                         name='map_lifecycle', output='screen',
                         parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': True}])
    nav_enabled = IfCondition(LaunchConfiguration('navigation'))
    social = arg('social').lower() in ('true', '1', 'yes')
    nav_source = Path(arg('nav_params')) if arg('nav_params') else bringup / 'config' / 'nav2_turtlebot4.yaml'
    nav_config = yaml.safe_load(nav_source.read_text())
    if not social:  # baseline: plain MPPI, people are only obstacles in the lidar scan
        for costmap in ('local_costmap', 'global_costmap'):
            nav_config[costmap][costmap]['ros__parameters']['social_layer']['enabled'] = False
        nav_config['controller_server']['ros__parameters']['FollowPath']['DynamicHumanCritic']['enabled'] = False
        nav_config['controller_server']['ros__parameters']['FollowPath']['SocialSpeedCritic']['enabled'] = False
        nav_config['controller_server']['ros__parameters']['FollowPath']['PassingSideCritic']['enabled'] = False
    follow = nav_config['controller_server']['ros__parameters']['FollowPath']
    follow['vx_max'] = float(arg('max_speed'))
    follow['SocialSpeedCritic']['max_speed'] = float(arg('max_speed'))
    if arg('people_source') == 'leg_tracker':
        # Tracked velocities are noisier than ground truth: widen predictions faster.
        follow['DynamicHumanCritic']['sigma_growth'] = 0.4
        for costmap in ('local_costmap', 'global_costmap'):
            params = nav_config[costmap][costmap]['ros__parameters']
            params['social_layer']['sigma_growth'] = 0.4
            # Lidar-only robot: tracked people are removed from the scan the obstacle layers see,
            # so they appear only as moving agents (with velocity), not as static obstacles.
            params['obstacle_layer']['scan']['topic'] = '/scan_filtered'
            params['obstacle_layer']['scan']['inf_is_valid'] = True
    # Single source of /people for the navigation stack: simulator ground truth or the
    # LiDAR leg tracker (same message, so the social layer and critic are unchanged).
    people_source = arg('people_source')
    if people_source == 'leg_tracker':
        people_adapter = Node(package='people_tracker', executable='leg_tracker', output='log',
                              parameters=[{'use_sim_time': True}])
    elif people_source == 'ground_truth':
        people_adapter = Node(package='people_tracker', executable='people_adapter', output='log')
    else:
        raise ValueError(f'people_source must be ground_truth or leg_tracker, not {people_source}')
    nav_nodes = ['controller_server', 'velocity_smoother', 'planner_server', 'behavior_server',
                 'bt_navigator']
    max_speed = float(arg('max_speed'))
    nav_config['velocity_smoother'] = {'ros__parameters': {
        'use_sim_time': True, 'smoothing_frequency': 20.0, 'scale_velocities': False,
        'feedback': 'OPEN_LOOP', 'max_velocity': [max_speed, 0.0, 1.5],
        'min_velocity': [-0.15, 0.0, -1.5], 'max_accel': [0.6, 0.0, 2.0],
        'max_decel': [-1.0, 0.0, -2.5], 'odom_topic': 'odom', 'odom_duration': 0.1,
        'deadband_velocity': [0.0, 0.0, 0.0], 'velocity_timeout': 1.0}}
    nav_params = str(run_directory / 'nav2_params.yaml')
    Path(nav_params).write_text(yaml.safe_dump(nav_config, sort_keys=False))
    navigation = [
        # Controller -> cmd_vel_nav -> velocity smoother (acceleration limits) -> cmd_vel.
        Node(package='nav2_controller', executable='controller_server', output='log',
             parameters=[nav_params], remappings=[('cmd_vel', 'cmd_vel_nav')], condition=nav_enabled),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='log', parameters=[nav_params],
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')], condition=nav_enabled),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='log',
             parameters=[nav_params], condition=nav_enabled),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='log',
             parameters=[nav_params], condition=nav_enabled),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='log',
             parameters=[nav_params], condition=nav_enabled),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', condition=nav_enabled,
             parameters=[{'autostart': True, 'node_names': nav_nodes, 'use_sim_time': True,
                          'bond_timeout': 10.0}]),
    ]
    mission = Node(
        package='social_nav_bringup', executable='robot_mission', output='screen',
        arguments=['--scenario', str(scenario), '--level', world_name,
                   '--robot-radius', str(ROBOT_RADIUS), '--humans-yield', str(humans_yield).lower(),
                   '--method', 'social_mppi' if social else 'mppi_baseline',
                   '--mission-timeout', arg('mission_timeout'), '--people-source', people_source],
        condition=IfCondition(LaunchConfiguration('auto_start')) if arg('navigation').lower() == 'true'
        else IfCondition('false'))

    # ---------------------------------------------------------------- visualisation
    rviz_enabled = IfCondition(LaunchConfiguration('rviz'))
    markers = Node(package='social_nav_bringup', executable='human_markers', output='screen',
                   arguments=['--scenario', str(scenario), '--robot-radius', str(ROBOT_RADIUS)],
                   condition=rviz_enabled)
    map_meta = yaml.safe_load(map_file.read_text())
    map_header = (assets / 'maps' / map_meta['image']).read_bytes().split(b'\n', 3)
    map_w, map_h = (int(v) for v in map_header[2].split())
    centre_x = map_meta['origin'][0] + map_w * map_meta['resolution'] / 2
    centre_y = map_meta['origin'][1] + map_h * map_meta['resolution'] / 2
    rviz_config = run_directory / 'simulation.rviz'
    rviz_config.write_text((bringup / 'rviz/simulation.rviz').read_text()
                           .replace('      X: 0\n', f'      X: {centre_x:.2f}\n', 1)
                           .replace('      Y: 0\n', f'      Y: {centre_y:.2f}\n', 1))
    rviz = Node(package='rviz2', executable='rviz2', output='log', condition=rviz_enabled,
                arguments=['-d', str(rviz_config)], parameters=[{'use_sim_time': True}])

    def start_gazebo(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='World generation failed'))]
        return [server, client, *navigation, mission]

    return [
        RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=start_gazebo)),
        RegisterEventHandler(OnProcessExit(target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server stopped'))])),
        RegisterEventHandler(OnProcessExit(target_action=client,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo window closed'))])),
        RegisterEventHandler(OnProcessExit(target_action=controller,
            on_exit=[EmitEvent(event=Shutdown(reason='People controller stopped'))])),
        loader, generator, controller, ready, robot_state, map_to_odom, people_adapter,
        map_server, map_lifecycle, markers, rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='level1',
                              choices=[f'level{n}' for n in range(1, 6)],
                              description='Social navigation level 1 (easy) .. 5 (hard)'),
        DeclareLaunchArgument('scenario', default_value='', description='Optional scenario YAML path'),
        DeclareLaunchArgument('behavior_tree_directory', default_value=''),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo client'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz with map, people and robot'),
        DeclareLaunchArgument('navigation', default_value='true', description='Run Nav2 for the TurtleBot 4'),
        DeclareLaunchArgument('auto_start', default_value='true',
                              description='Send the level GOAL to Nav2 automatically and log the run'),
        DeclareLaunchArgument('nav_params', default_value='', description='Optional Nav2 params YAML'),
        DeclareLaunchArgument('max_speed', default_value='0.65',
                              description='Robot forward speed limit [m/s] (real TurtleBot 4: 0.306)'),
        DeclareLaunchArgument('mission_timeout', default_value='300',
                              description='Simulated-run limit for robot_mission [s]'),
        DeclareLaunchArgument('people_source', default_value='leg_tracker',
                              choices=['ground_truth', 'leg_tracker'],
                              description='Where /people comes from: simulator or LiDAR leg tracker'),
        DeclareLaunchArgument('social', default_value='true',
                              description='true: MPPI + social costmap layer + dynamic human critic; '
                                          'false: plain MPPI baseline'),
        DeclareLaunchArgument('humans_yield', default_value='false',
                              description='true: people also step aside for the robot; '
                                          'false: only the robot avoids (people ignore it)'),
        DeclareLaunchArgument('stuck_goal_timeout', default_value='8.0',
                              description='Seconds without progress before a person skips a goal'),
        DeclareLaunchArgument('runtime_dir', default_value=os.environ.get('SIM_RUNTIME_DIR', '.runtime')),
        OpaqueFunction(function=launch_simulation),
    ])
