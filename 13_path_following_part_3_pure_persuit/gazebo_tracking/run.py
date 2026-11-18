#!/usr/bin/env python3
"""Run a single Gazebo path-tracking experiment using ros1_car."""

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from tempfile import TemporaryDirectory
from uuid import uuid4


PROJECT = Path(__file__).resolve().parent
CARLA = PROJECT.parents[2] / 'carla_town_loop_tracking'
CONTAINER_METRICS = Path('/tmp/ugv-gazebo-metrics')
DEFAULT_ROUTE = PROJECT.parent / 'waypoint_loader/waypoints/s_curve.csv'
CONTROLLERS = ('pure_pursuit', 'stanley')


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError('expected a positive integer')
    return number


def road_file(value):
    route = Path(value).resolve()
    if not route.is_file():
        raise argparse.ArgumentTypeError('expected an existing route CSV')
    return route


def parser_for(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--container', default='ros1_car')
    parser.add_argument('--in-container', action='store_true',
                        help='run directly in the container after sourcing its ROS environment')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--half-width', type=float, default=1.75,
                        help='tracking corridor half-width in metres (default: 1.75)')
    return parser


def check_args(parser, args, routes):
    if not math.isfinite(args.half_width) or args.half_width <= 0:
        parser.error('--half-width must be positive')
    if len({route.stem for route in routes}) != len(routes):
        parser.error('route filenames must have distinct stems')


def run_trials(args, routes, trials, prefix):
    metrics_dir = CONTAINER_METRICS if args.in_container else CARLA
    missing = [metrics_dir / name for name in ('progress.py', 'evaluate.py')
               if not (metrics_dir / name).is_file()]
    if missing:
        raise SystemExit(f'Missing shared metrics module: {missing[0]}. See the chapter README.')
    output = (args.output_dir or PROJECT / 'results' /
              f'{prefix}_{datetime.now():%Y%m%d_%H%M%S}').resolve()
    output.mkdir(parents=True)
    references = output / 'routes'
    references.mkdir()
    for route in routes:
        shutil.copyfile(route, references / f'{route.stem}.csv')
    (output / 'batch.json').write_text(json.dumps({
        'platform': 'Gazebo',
        'route_labels': {route.stem: 'S形路线' if route.stem == 's_curve' else route.stem for route in routes},
        'half_width_m': args.half_width,
    }, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

    docker = ['docker', 'exec', args.container]
    print(f'Output: {output}', flush=True)
    with TemporaryDirectory(prefix='ugv-gazebo-') as directory:
        stage = Path(directory)
        for script in (PROJECT / 'ros_trial.py', PROJECT / 'trial.launch',
                       metrics_dir / 'progress.py', metrics_dir / 'evaluate.py'):
            shutil.copyfile(script, stage / script.name)
        shutil.copytree(references, stage / 'routes')
        work = stage if args.in_container else Path(f'/tmp/ugv-gazebo-{uuid4().hex}')
        if not args.in_container:
            subprocess.run(['docker', 'cp', directory, f'{args.container}:{work}'], check=True)
        try:
            with (output / 'summary.csv').open('w', newline='') as summary:
                writer = csv.writer(summary)
                for trial_index, (route, speed, controller, repeat) in enumerate(trials):
                    label = f'{route.stem}_{controller}_{speed}kmh_r{repeat:02d}'
                    print(f'Running {label}', flush=True)
                    log = output / f'{label}.log'
                    trial_csv = f'{label}.csv'
                    track = output / trial_csv
                    recorded = track if args.in_container else work / trial_csv
                    row_file = stage / 'trial_summary.csv'
                    pid_file = str(work / 'trial.pid')
                    arguments = [
                        str(work / 'ros_trial.py'),
                        '--route', str(work / 'routes' / f'{route.stem}.csv'), '--route-name', route.stem,
                        '--controller', controller, '--speed', str(speed), '--repeat', str(repeat),
                        '--half-width', str(args.half_width), '--trajectory', str(recorded),
                        '--summary', str(work / row_file.name),
                    ]
                    if args.in_container:
                        command = [sys.executable, '-B', '-u', *arguments]
                    else:
                        command = docker + [
                            'bash', '-c',
                            'source /opt/ros/noetic/setup.bash && source /root/car_ws/devel/setup.bash '
                            '&& echo "$$" > "$1" && shift && exec python3 -B -u "$@"',
                            'bash', pid_file, *arguments,
                        ]
                    with log.open('w') as stream:
                        try:
                            result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
                        except KeyboardInterrupt:
                            if not args.in_container:
                                subprocess.run(docker + ['sh', '-c', 'kill -INT "$(cat "$1")"', 'sh', pid_file],
                                               check=False)
                            raise
                    if not args.in_container:
                        subprocess.run(['docker', 'cp', f'{args.container}:{recorded}', str(track)],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                       check=result.returncode == 0)
                    if result.returncode:
                        raise RuntimeError(f'Gazebo trial failed. See {log}')
                    if not args.in_container:
                        subprocess.run(['docker', 'cp', f'{args.container}:{work / row_file.name}', str(row_file)],
                                       check=True)
                    with row_file.open(newline='') as source:
                        fields, row = list(csv.reader(source))
                    if trial_index == 0:
                        writer.writerow(fields)
                    writer.writerow(row)
                    summary.flush()
                    metrics = dict(zip(fields, row))
                    print(f"  completed={metrics['completed']}, outside_lane={metrics['outside_lane']}, "
                          f"RMSE={metrics['rmse_m'] or 'n/a'} m", flush=True)
        finally:
            if not args.in_container:
                subprocess.run(docker + ['rm', '-rf', str(work)], check=False)
    print(f'Summary: {output / "summary.csv"}')


def main():
    parser = parser_for(__doc__)
    parser.add_argument('--route', type=road_file, default=DEFAULT_ROUTE)
    parser.add_argument('--controller', choices=CONTROLLERS, required=True)
    parser.add_argument('--speed', type=positive_int, required=True, help='target speed in km/h')
    args = parser.parse_args()
    routes = [args.route]
    check_args(parser, args, routes)
    run_trials(args, routes, [(args.route, args.speed, args.controller, 1)], 'trial')


if __name__ == '__main__':
    main()
