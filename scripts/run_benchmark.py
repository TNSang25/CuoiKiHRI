#!/usr/bin/env python3
"""Run every level with the social method and the MPPI baseline, headless, and tabulate results.

  source scripts/env.bash
  python3 scripts/run_benchmark.py                    # levels 1-5, both methods, 1 run each
  python3 scripts/run_benchmark.py --levels 2 5 --runs 3 --methods social

Each run writes .runtime/results/mission_*.json (see robot_mission); this script collects them
into .runtime/results/benchmark_<time>.csv and prints a Markdown table.
"""
import argparse
import csv
import json
import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[1]
RESULTS = Path(os.environ.get('SIM_RESULTS_DIR', WORKSPACE / '.runtime' / 'results'))
METHODS = {'social': 'true', 'baseline': 'false'}
FIELDS = ['level', 'method', 'people_source', 'run', 'success', 'time_s', 'path_length_m', 'min_clearance_to_person_m',
          'personal_space_intrusion_s', 'collisions', 'robot_moving_collisions']


def run_once(level, method, timeout, extra):
    before = set(RESULTS.glob(f'mission_level{level}_*.json'))
    cmd = ['ros2', 'launch', 'social_nav_bringup', 'simulation.launch.py', f'world:=level{level}',
           f'social:={METHODS[method]}', 'gui:=false', 'rviz:=false', *extra]
    log = (RESULTS / f'benchmark_level{level}_{method}.log').open('w')
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    deadline = time.monotonic() + timeout
    found = None
    try:
        while time.monotonic() < deadline and proc.poll() is None:
            time.sleep(2)
            new = set(RESULTS.glob(f'mission_level{level}_*.json')) - before
            if new:
                found = max(new, key=lambda p: p.stat().st_mtime)
                time.sleep(1)
                break
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        subprocess.run(['pkill', '-KILL', '-x', 'gzserver'], check=False)
        time.sleep(3)
    if found is None:
        return {'level': f'level{level}', 'status': 'no_result', 'success': False}
    return json.loads(found.read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--levels', type=int, nargs='+', default=[1, 2, 3, 4, 5])
    parser.add_argument('--methods', nargs='+', choices=sorted(METHODS), default=['social', 'baseline'])
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--timeout', type=float, default=420.0, help='Wall seconds per run')
    parser.add_argument('extra', nargs='*', help='Extra launch arguments, e.g. humans_yield:=true')
    args = parser.parse_args()
    RESULTS.mkdir(parents=True, exist_ok=True)
    rows = []
    for level in args.levels:
        for method in args.methods:
            for run in range(1, args.runs + 1):
                print(f'level{level} {method} run {run} ...', flush=True)
                result = run_once(level, method, args.timeout, args.extra)
                row = {k: result.get(k) for k in FIELDS}
                row.update(level=f'level{level}', method=method, run=run)
                row['people_source'] = result.get('people_source', 'ground_truth')
                rows.append(row)
                print('  ' + json.dumps(row), flush=True)
    out = RESULTS / f'benchmark_{datetime.now():%Y%m%d_%H%M%S}.csv'
    with out.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print(f'\n| {" | ".join(FIELDS)} |\n|{"---|" * len(FIELDS)}')
    for row in rows:
        print('| ' + ' | '.join(str(row[k]) for k in FIELDS) + ' |')
    print(f'\nCSV: {out}')


if __name__ == '__main__':
    main()
