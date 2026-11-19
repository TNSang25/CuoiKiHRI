"""Generate basic navigation trees from a scenario without an LLM service."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def build_tree(agent_id, behavior):
    kind = behavior.get('type', 'Regular')
    if kind not in {'Regular', 'Impassive', 'Surprised', 'Curious', 'Scared', 'Threatening'}:
        raise ValueError(f'Unsupported behavior for agent {agent_id}: {kind}')
    root = ET.Element('root', BTCPP_format='4', main_tree_to_execute=f'Agent{agent_id}')
    tree = ET.SubElement(root, 'BehaviorTree', ID=f'Agent{agent_id}')
    fallback = ET.SubElement(tree, 'Fallback')
    if kind not in {'Regular', 'Impassive'}:
        reaction = ET.SubElement(fallback, 'Sequence')
        ET.SubElement(reaction, 'IsRobotVisible', agent_id='{id}', distance='5.0')
        timer = ET.SubElement(reaction, 'Inverter')
        ET.SubElement(timer, 'TimeExpiredCondition',
                      seconds=str(float(behavior.get('duration', 10.0))), ts='{dt}',
                      only_once=str(bool(behavior.get('once', True))).lower())
        ports = {
            'agent_id': '{id}', 'time_step': '{dt}',
            'beh_duration': str(float(behavior.get('duration', 10.0))),
            'only_once': str(bool(behavior.get('once', True))).lower(),
        }
        action = {'Surprised': 'SurprisedNav', 'Curious': 'CuriousNav',
                  'Scared': 'ScaredNav', 'Threatening': 'ThreateningNav'}[kind]
        if kind == 'Curious':
            ports.update(agent_vel=str(float(behavior.get('vel', 0.6))),
                         stop_distance=str(float(behavior.get('dist', 1.0))))
        elif kind == 'Scared':
            ports.update(runaway_vel=str(float(behavior.get('vel', 1.5))),
                         safe_distance=str(float(behavior.get('safe_distance', 2.0))))
        elif kind == 'Threatening':
            ports['goal_dist'] = str(float(behavior.get('dist', 1.0)))
        ET.SubElement(reaction, action, **ports)
    navigation = ET.SubElement(fallback, 'Sequence')
    inverter = ET.SubElement(navigation, 'Inverter')
    ET.SubElement(inverter, 'IsGoalReached', agent_id='{id}')
    ET.SubElement(navigation, 'RegularNav', agent_id='{id}', time_step='{dt}')
    ET.SubElement(fallback, 'UpdateGoal', agent_id='{id}')
    ET.indent(root)
    return ET.tostring(root, encoding='unicode') + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--force', action='store_true', help='Replace existing trees')
    args = parser.parse_args()
    try:
        params = yaml.safe_load(args.scenario.read_text())['bot_agent_loader']['ros__parameters']
        base = params['yaml_base_name']
        if not base or Path(base).name != base:
            raise ValueError('yaml_base_name must be a plain file name')
        outputs, ids = [], set()
        for name in params['agents']:
            agent = params[name]
            agent_id = int(agent['id'])
            if agent_id < 0 or agent_id in ids:
                raise ValueError('Agent IDs must be unique nonnegative integers')
            ids.add(agent_id)
            destination = args.output_dir / f'{base}__agent_{agent_id}_bt.xml'
            if destination.exists() and not args.force:
                raise FileExistsError(f'{destination} already exists; use --force to replace it')
            outputs.append((destination, build_tree(agent_id, agent.get('behavior', {}))))
        if not outputs:
            raise ValueError('Scenario has no agents')
        args.output_dir.mkdir(parents=True, exist_ok=True)
        for destination, content in outputs:
            destination.write_text(content)
            print(destination)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        parser.exit(1, f'{error}\n')


if __name__ == '__main__':
    main()
