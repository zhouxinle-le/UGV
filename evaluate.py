#!/usr/bin/env python3
"""Summarize CARLA trajectory CSVs with a shared route-progress window."""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np


def evaluate(reference, samples, trim):
    route_length = np.linalg.norm(np.diff(reference, axis=0), axis=1).cumsum()[-1]
    time, x, y, speed, target_speed, steering, _, progress, error = samples.T
    endpoint = float(np.linalg.norm(np.array([x[-1], y[-1]]) - reference[-1]))
    stopped = time >= time[-1] - 1.0 - 1e-9
    completed = (endpoint <= 1.0 and progress[-1] >= route_length - 1.0 and time[-1] - time[0] >= 1.0
                 and np.all(speed[stopped] < 0.1) and np.all(target_speed[stopped] < 0.05))
    result = {
        'completed': completed,
        'elapsed_s': round(float(time[-1] - time[0]), 3),
        'endpoint_error_m': round(endpoint, 3),
        'rmse_m': '', 'max_error_m': '', 'mean_speed_kmh': '', 'steering_rate_rms_deg_s': '',
    }

    start = np.flatnonzero(progress >= trim)
    if not start.size:
        return result
    end = np.flatnonzero((progress >= route_length - trim) & (np.arange(len(time)) > start[0]))
    if not end.size:
        return result
    window = slice(start[0], end[0] + 1)
    dt = np.diff(time[window])
    duration = dt.sum()

    def average(values):
        return np.dot((values[:-1] + values[1:]) / 2, dt) / duration

    error = error[window]
    rate = np.degrees(np.diff(steering[window])) / dt
    result.update(
        rmse_m=round(float(np.sqrt(average(error**2))), 3),
        max_error_m=round(float(error.max()), 3),
        mean_speed_kmh=round(float(average(speed[window]) * 3.6), 3),
        steering_rate_rms_deg_s=round(float(np.sqrt(np.dot(rate**2, dt) / duration)), 3),
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('route', type=Path)
    parser.add_argument('trajectories', type=Path, nargs='+')
    parser.add_argument('--trim', type=float, default=10.0, help='metres excluded at each end')
    args = parser.parse_args()
    route = np.loadtxt(args.route, delimiter=',', usecols=(0, 1), ndmin=2)
    route_length = np.linalg.norm(np.diff(route, axis=0), axis=1).sum()
    if not 0 <= 2 * args.trim < route_length:
        parser.error('the trimming distance must leave a nonempty interval')

    writer = csv.DictWriter(sys.stdout, fieldnames=(
        'trajectory', 'completed', 'elapsed_s', 'endpoint_error_m', 'rmse_m',
        'max_error_m', 'mean_speed_kmh', 'steering_rate_rms_deg_s',
    ))
    writer.writeheader()
    for trajectory in args.trajectories:
        samples = np.loadtxt(trajectory, delimiter=',', skiprows=1, ndmin=2)
        if len(samples) < 2:
            writer.writerow({'trajectory': trajectory.name, 'completed': False})
            continue
        writer.writerow({'trajectory': trajectory.name, **evaluate(route, samples, args.trim)})


if __name__ == '__main__':
    main()
