#!/usr/bin/env python3
"""Run and summarize the Town04 assignment matrix."""

import argparse
import csv
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

if __package__:
    from .evaluate import evaluate
    from .run import CASES, DEFAULT_MAP
else:
    from evaluate import evaluate
    from run import CASES, DEFAULT_MAP

PROJECT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--carla-port', type=int, default=2000)
    parser.add_argument('--ros-port', type=int, default=9090)
    parser.add_argument('--container', default='ros1_car')
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be at least one')

    output = args.output_dir or (PROJECT / 'results' / f'matrix_{datetime.now():%Y%m%d_%H%M%S}')
    output.mkdir(parents=True)
    fields = ('route', 'controller', 'target_speed_kmh', 'repeat', 'completed', 'outside_lane',
              'elapsed_s', 'endpoint_error_m', 'rmse_m', 'max_error_m', 'mean_speed_kmh',
              'steering_rate_rms_deg_s')
    with (output / 'summary.csv').open('w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for case in CASES:
            for speed in (5, 10, 15):
                for controller in ('pure_pursuit', 'stanley'):
                    for repeat in range(1, args.repeats + 1):
                        label = f'{case}_{controller}_{speed}kmh_r{repeat:02}'
                        track = output / f'{label}.csv'
                        log_path = output / f'{label}.log'
                        print(f'Running {label}', flush=True)
                        command = [
                            sys.executable, '-B', '-u', str(PROJECT / 'run.py'),
                            '--carla-port', str(args.carla_port), '--ros-port', str(args.ros_port),
                            '--container', args.container, '--case', case, '--controller', controller,
                            '--speed', str(speed), '--output', str(track), '--no-rendering',
                        ]
                        with log_path.open('w') as log:
                            trial = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
                        if not track.is_file():
                            raise RuntimeError(f'Trial did not start. See {log_path.resolve()}')
                        samples = np.loadtxt(track, delimiter=',', skiprows=1, ndmin=2)
                        if len(samples) < 2:
                            raise RuntimeError(f'Trial produced no usable samples. See {log_path.resolve()}')
                        route = np.loadtxt(PROJECT / 'routes' / f'{DEFAULT_MAP}_{case}.csv',
                                           delimiter=',', usecols=(0, 1))
                        metrics = evaluate(route, samples, trim=10.0)
                        metrics['completed'] = bool(metrics['completed'] and trial.returncode == 0)
                        outside_lane = bool(samples[:, 6].any())
                        writer.writerow(dict(route=case, controller=controller, target_speed_kmh=speed,
                                             repeat=repeat, outside_lane=outside_lane, **metrics))
                        file.flush()
                        print(f'  completed={metrics["completed"]}, outside_lane={outside_lane}, '
                              f'RMSE={metrics["rmse_m"]} m', flush=True)
    print(f'Summary: {(output / "summary.csv").resolve()}')


if __name__ == '__main__':
    main()
