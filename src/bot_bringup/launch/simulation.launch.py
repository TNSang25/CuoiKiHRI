"""Launch local people behavior, Gazebo simulation and optional evaluation."""
from pathlib import Path
import os
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


def launch_simulation(context):
    assets = Path(get_package_share_directory('bot_gazebo'))
    world_name = LaunchConfiguration('world').perform(context)
    scenario_arg = LaunchConfiguration('scenario').perform(context)
    scenario = Path(scenario_arg) if scenario_arg else assets / 'scenarios' / f'agents_{world_name}.yaml'
    configuration = yaml.safe_load(scenario.read_text())['bot_agent_loader']['ros__parameters']
    agents = configuration['agents']
    if not agents:
        raise ValueError('The scenario must contain at least one agent')
    ids = [configuration[name]['id'] for name in agents]
    if len(ids) != len(set(ids)):
        raise ValueError('Agent IDs must be unique')
    tree_argument = LaunchConfiguration('behavior_tree_directory').perform(context)
    tree_directory = Path(tree_argument) if tree_argument else assets / 'behavior_trees'
    base_name = configuration['yaml_base_name']
    for agent_id in ids:
        tree_file = tree_directory / f'{base_name}__agent_{agent_id}_bt.xml'
        if not tree_file.is_file():
            raise FileNotFoundError(f'Missing behavior tree: {tree_file}')

    runtime = Path(LaunchConfiguration('runtime_dir').perform(context)).resolve()
    run_directory = runtime / 'worlds' / uuid.uuid4().hex[:8]
    run_directory.mkdir(parents=True, exist_ok=True)
    generated = run_directory / 'generatedWorld.world'
    world_tree = ET.parse(assets / 'worlds' / f'{world_name}.world')
    world = world_tree.getroot().find('world')
    world.find('physics/max_step_size').text = '0.01'
    world.find('physics/real_time_update_rate').text = '100'
    world.append(ET.fromstring('''
      <model name="robot">
        <static>true</static><pose>0 0 0.3 0 0 0</pose>
        <link name="base_link">
          <collision name="collision"><geometry><cylinder>
            <radius>0.35</radius><length>0.6</length>
          </cylinder></geometry></collision>
          <visual name="visual"><geometry><cylinder>
            <radius>0.35</radius><length>0.6</length>
          </cylinder></geometry><material>
            <ambient>0.05 0.3 0.9 1</ambient><diffuse>0.05 0.3 0.9 1</diffuse>
          </material></visual>
        </link>
      </model>'''))
    base_world = run_directory / f'{world_name}.world'
    world_tree.write(base_world, encoding='utf-8', xml_declaration=True)

    loader = Node(package='bot_agent_control', executable='bot_agent_loader',
                  output='screen', parameters=[str(scenario)])
    generator = Node(
        package='bot_gazebo', executable='bot_world_generator', output='screen',
        parameters=[{
            'base_world': str(base_world), 'robot_name': 'robot',
            'use_gazebo_obs': True, 'use_collision': False, 'update_rate': 20.0,
            'global_frame_to_publish': 'map', 'use_navgoal_to_start': False,
            'ignore_models': 'aws_robomaker_warehouse_GroundB_01_001 ground_plane FloorB_01_001',
        }])
    controller = Node(
        package='bot_agent_control', executable='bot_agent_control', output='screen',
        parameters=[{'use_sim_time': True, 'behavior_tree_directory': str(tree_directory)}])
    server = ExecuteProcess(
        cmd=['gzserver', '--verbose', str(generated),
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    client = ExecuteProcess(cmd=['gzclient'], output='screen',
                            condition=IfCondition(LaunchConfiguration('gui')))
    wait_script = Path(get_package_prefix('bot_bringup')) / 'lib/bot_bringup/wait_world'
    ready = ExecuteProcess(cmd=[sys.executable, str(wait_script), str(generated), str(len(agents))],
                           output='screen')
    results = runtime / 'results'
    results.mkdir(parents=True, exist_ok=True)
    evaluator = Node(
        package='bot_evaluation', executable='bot_evaluation_node', output='screen',
        parameters=[str(Path(get_package_share_directory('bot_evaluation')) / 'config/metrics.yaml'),
                    {'result_path': str(results), 'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('evaluation')))

    def start_gazebo(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='World generation failed'))]
        return [server, client]

    return [
        RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=start_gazebo)),
        RegisterEventHandler(OnProcessExit(target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server stopped'))])),
        RegisterEventHandler(OnProcessExit(target_action=client,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo window closed'))])),
        RegisterEventHandler(OnProcessExit(target_action=controller,
            on_exit=[EmitEvent(event=Shutdown(reason='People controller stopped'))])),
        loader, generator, controller, evaluator, ready,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse', choices=['warehouse', 'house']),
        DeclareLaunchArgument('scenario', default_value='', description='Optional scenario YAML path'),
        DeclareLaunchArgument('behavior_tree_directory', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('evaluation', default_value='true'),
        DeclareLaunchArgument('runtime_dir', default_value=os.environ.get('BOT_RUNTIME_DIR', '.runtime')),
        OpaqueFunction(function=launch_simulation),
    ])
