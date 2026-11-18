#!/usr/bin/env python3
"""Overlay CARLA reference paths and one actual trial per method and speed."""

import argparse
import csv
import json
from pathlib import Path
import sys

import matplotlib as mpl

mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import fontManager
from matplotlib.patches import Rectangle
import numpy as np


METHODS = (
    ('pure_pursuit', '纯点跟踪', '#1769AA', '-'),
    ('stanley', 'stanley方法', '#BA4900', '--'),
)


def zoom_bounds(reference):
    # Select the tightest reference bend without using either method's result.
    spacing = np.linalg.norm(np.diff(reference, axis=0), axis=1)
    stations = np.r_[0, spacing.cumsum()]
    if np.any(spacing <= 0):
        raise ValueError('Reference path contains repeated adjacent positions.')
    yaw = np.unwrap(np.arctan2(*np.gradient(reference, axis=0).T[::-1]))
    curvature = np.abs(np.gradient(yaw, stations))
    curvature[:3] = curvature[-3:] = 0
    center = int(curvature.argmax())
    selected = reference[np.abs(stations - stations[center]) <= 5]
    lower, upper = selected.min(axis=0), selected.max(axis=0)
    midpoint = (lower + upper) / 2
    side = max(float((upper - lower).max()) + 12, 24)
    return midpoint[0] - side / 2, midpoint[0] + side / 2, midpoint[1] - side / 2, midpoint[1] + side / 2


def square_bounds(points):
    lower, upper = points.min(axis=0), points.max(axis=0)
    midpoint = (lower + upper) / 2
    side = float((upper - lower).max()) * 1.12
    return midpoint[0] - side / 2, midpoint[0] + side / 2, midpoint[1] - side / 2, midpoint[1] + side / 2


def clipped_path(points, bounds):
    """Clip line segments to the panel while retaining crossing endpoints."""
    result = []
    for first, last in zip(points[:-1], points[1:]):
        delta = last - first
        lower, upper = 0.0, 1.0
        for coordinate, minimum, maximum in ((0, bounds[0], bounds[1]), (1, bounds[2], bounds[3])):
            if delta[coordinate] == 0:
                if not minimum <= first[coordinate] <= maximum:
                    upper = -1
                    break
            else:
                a, b = sorted(((minimum - first[coordinate]) / delta[coordinate],
                               (maximum - first[coordinate]) / delta[coordinate]))
                lower, upper = max(lower, a), min(upper, b)
        if lower > upper:
            continue
        a, b = first + lower * delta, first + upper * delta
        if result and not np.allclose(result[-1], a, rtol=0, atol=1e-9):
            result.append(np.array([np.nan, np.nan]))
        if not result or not np.allclose(result[-1], a, rtol=0, atol=1e-9):
            result.append(a)
        result.append(b)
    return np.array(result).reshape(-1, 2)


def load_trajectories(batch, rows):
    trajectories = {}
    for row in rows:
        filename = (f"{row['route']}_{row['controller']}_{int(row['target_speed_kmh'])}kmh"
                    f"_r{int(row['repeat']):02}.csv")
        with (batch / filename).open(newline='') as stream:
            samples = csv.DictReader(stream)
            points = np.array([(float(sample['x']), float(sample['y'])) for sample in samples])
        if len(points) < 2 or not np.isfinite(points).all():
            raise ValueError(f'Trajectory contains too few or invalid positions: {filename}')
        key = int(row['target_speed_kmh']), row['controller'], int(row['repeat'])
        if key in trajectories:
            raise ValueError(f'Duplicate trial: {filename}')
        trajectories[key] = points
    return trajectories


def draw_paths(reference, trajectories, route_name, qa_file):
    from audit_panel_alignment import require_matplotlib_panel_alignment

    speeds = sorted({key[0] for key in trajectories})
    fig, axes = plt.subplots(2, len(speeds), squeeze=False,
                             figsize=(max(120, 68 * len(speeds)) / 25.4, 146 / 25.4))
    fig.subplots_adjust(left=.075, right=.985, bottom=.12, top=.88, hspace=.36, wspace=.22)
    zoom = zoom_bounds(reference)
    full = square_bounds(np.concatenate([reference, *trajectories.values(),
                                         np.array([[zoom[0], zoom[2]], [zoom[1], zoom[3]]])]))
    manifest = {}
    for col, speed in enumerate(speeds):
        for row, limits in enumerate((full, zoom)):
            ax = axes[row, col]
            for method, title, color, linestyle in METHODS:
                runs = sorted((key, points) for key, points in trajectories.items()
                              if key[:2] == (speed, method))
                for index, (_, points) in enumerate(runs):
                    visible = clipped_path(points, limits)
                    ax.plot(visible[:, 0], visible[:, 1], color=color, linestyle=linestyle,
                            linewidth=.85, alpha=.85,
                            label=title if index == 0 else None, zorder=2)
            visible = clipped_path(reference, limits)
            ax.plot(visible[:, 0], visible[:, 1], color='#222222', linestyle=':',
                    linewidth=1.1, label='参考路径', zorder=3)
            ax.set(xlim=limits[:2], ylim=limits[2:], xlabel='横向坐标（米）', ylabel='纵向坐标（米）')
            ax.set_aspect('equal', adjustable='box')
            ax.set_title(f'{speed}千米/小时的{route_name}' if row == 0 else '相同弯道的局部放大',
                         fontsize=7, pad=6)
            ax.tick_params(length=2.5, width=.55, pad=2)
            if row == 0:
                ax.add_patch(Rectangle((zoom[0], zoom[2]), zoom[1] - zoom[0], zoom[3] - zoom[2],
                                       fill=False, edgecolor='#707070', linewidth=.65, linestyle='-.', zorder=1))
            if row == 0:
                ax.legend(loc='lower right',
                          fontsize=5.6, frameon=True,
                          facecolor='white', edgecolor='#CCCCCC', framealpha=1,
                          handlelength=1.6, handletextpad=.35)
            else:
                ax.legend(loc='upper right', fontsize=5.6, frameon=True,
                          facecolor='white', edgecolor='#CCCCCC', framealpha=1,
                          handlelength=1.6, handletextpad=.35)
        manifest[str(speed)] = {
            'overview_bounds': full, 'zoom_bounds': zoom,
            'runs': {title: [key[2] for key in trajectories if key[:2] == (speed, method)]
                     for method, title, _, _ in METHODS},
        }
    require_matplotlib_panel_alignment(fig, json_out=qa_file, strict=True)
    return fig, manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path, help='CARLA batch containing summary.csv and trajectory CSVs')
    parser.add_argument('--output-dir', type=Path, help='default: analysis/outputs/<batch name>')
    parser.add_argument('--routes-dir', type=Path, default=Path(__file__).resolve().parents[1] / 'routes',
                        help='reference directory containing Town04_Opt_<route>.csv')
    parser.add_argument('--repeat', type=int, default=1, help='trial number for both methods (default: 1)')
    parser.add_argument('--figure-skill', type=Path, default=Path.home() / '.codex/skills/nature-figure')
    args = parser.parse_args()
    sys.path.insert(0, str(args.figure_skill / 'scripts'))

    fontManager.addfont('/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf')
    mpl.rcParams.update({
        'font.family': ['Liberation Sans', 'Droid Sans Fallback', 'sans-serif'],
        'font.size': 6.5, 'axes.labelsize': 6.5, 'axes.linewidth': .55,
        'axes.unicode_minus': False, 'pdf.fonttype': 42, 'svg.fonttype': 'none',
        'savefig.facecolor': 'white',
    })
    batch = args.batch.resolve()
    with (batch / 'summary.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        parser.error('summary.csv has no trial records.')
    if args.repeat <= 0:
        parser.error('--repeat must be positive.')
    conditions = {(row['route'], int(row['target_speed_kmh'])) for row in rows}
    selected = [row for row in rows if int(row['repeat']) == args.repeat]
    for case, speed in conditions:
        found = {row['controller'] for row in selected
                 if row['route'] == case and int(row['target_speed_kmh']) == speed}
        if found != {method[0] for method in METHODS}:
            parser.error(f'{case}, {speed} km/h does not contain both methods for trial {args.repeat}.')
    rows = selected
    labels = {'forward': '大环路线', 'clockwise': '小环路线'}
    output = (args.output_dir or Path(__file__).resolve().parent / 'outputs' / batch.name).resolve()
    output.mkdir(parents=True, exist_ok=True)
    qa = output / '.qa'
    qa.mkdir(exist_ok=True)
    for case in dict.fromkeys(row['route'] for row in rows):
        reference_file = args.routes_dir.resolve() / f'Town04_Opt_{case}.csv'
        reference = np.loadtxt(reference_file, delimiter=',', usecols=(0, 1), ndmin=2)
        trajectories = load_trajectories(batch, [row for row in rows if row['route'] == case])
        stem = f'CARLA_{case}_实际路径对比'
        fig, manifest = draw_paths(reference, trajectories, labels[case], qa / f'{stem}.alignment.json')
        for condition in manifest.values():
            condition['reference_file'] = str(reference_file)
        fig.savefig(output / f'{stem}.pdf')
        fig.savefig(output / f'{stem}.svg')
        fig.savefig(output / f'{stem}.png', dpi=300)
        fig.savefig(output / f'{stem}.tiff', dpi=600, pil_kwargs={'compression': 'tiff_lzw'})
        print(output / f'{stem}.pdf')
        plt.close(fig)
        (qa / f'{stem}.sources.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2),
                                                encoding='utf-8')


if __name__ == '__main__':
    main()
