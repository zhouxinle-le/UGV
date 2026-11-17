"""Plot mean metrics with standard-deviation error bars."""

import json
import math
import sys
from pathlib import Path

import matplotlib as mpl

mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import fontManager
from matplotlib.path import Path as MplPath
from matplotlib.ticker import MaxNLocator

from audit_panel_alignment import require_matplotlib_panel_alignment


ROUTES = (('forward', '大环'), ('clockwise', '小环'))
METRICS = (
    ('rmse_m', '跟踪误差', '平均跟踪误差（米）'),
    ('mean_speed_kmh', '平均车速', '实测平均车速（千米/小时）'),
    ('steering_rate_rms_deg_s', '转向平滑性', '平均转向变化率（度/秒）'),
)
CONTROLLERS = (
    ('pure_pursuit', '纯点跟踪', '#1769AA', 'o', '-'),
    ('stanley', 'stanley方法', '#BA4900', 's', (0, (3.5, 1.8))),
)
STEM = 'Town04_三项指标对比'


def draw_panel(ax, route_name, groups, metric, title, y_label):
    ax.set_title(f'{route_name}的{title}', fontsize=7, pad=5)
    ax.set_xlabel('目标速度（千米/小时）')
    ax.set_ylabel(y_label)
    speeds = sorted({entry['speed'] for entry in groups})
    if not speeds:
        ax.set_xticks([])
        ax.set_yticks([])
        ax.text(.5, .5, '暂无试次', transform=ax.transAxes, ha='center', va='center')
        return []

    span = max(1, speeds[-1] - speeds[0])
    ax.set_xlim(speeds[0] - .18 * span, speeds[-1] + .18 * span)
    ax.set_xticks(speeds)
    ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
    by_condition = {(group['speed'], group['controller']): group['metrics'][metric]
                    for group in groups}
    extents = [(mean - (sd or 0), mean + (sd or 0))
               for mean, sd in by_condition.values() if mean is not None]
    if not extents:
        ax.text(.5, .5, '暂无有效数据', transform=ax.transAxes, ha='center', va='center')
        return []
    low = min(limits[0] for limits in extents)
    high = max(limits[1] for limits in extents)
    spread = max(high - low, .10 * abs(high), .001)
    ax.set_ylim(max(-.045 * spread, low - .24 * spread), high + .40 * spread)

    for controller, label, color, marker, pattern in CONTROLLERS:
        summaries = [by_condition.get((speed, controller), (None, None)) for speed in speeds]
        if not any(mean is not None for mean, _ in summaries):
            continue
        means = [math.nan if mean is None else mean for mean, _ in summaries]
        sd = [math.nan if sd is None else sd for _, sd in summaries]
        ax.errorbar(speeds, means, yerr=sd, color=color, label=label, linestyle=pattern,
                    linewidth=1, marker=marker, markersize=3.1, markeredgewidth=.65,
                    markerfacecolor=color if marker == 'o' else 'none',
                    elinewidth=.65, capsize=1.4, capthick=.65, zorder=3)

    annotations = []
    for speed in speeds:
        available = [
            (mean, color, sd)
            for controller, _, color, _, _ in CONTROLLERS
            for mean, sd in [by_condition.get((speed, controller), (None, None))]
            if mean is not None
        ]
        available.sort()
        for index, (mean, color, sd) in enumerate(available):
            above = index == len(available) - 1
            direction = 1 if above else -1
            annotations.append((f'{mean:.3f}', speed, mean, sd or 0, direction, color))

    key = ax.legend(loc='upper left',
                    fontsize=5.9, frameon=True, framealpha=1, facecolor='white',
                    edgecolor='#B5B9BE', fancybox=False, borderpad=.35, borderaxespad=.45,
                    labelspacing=.28, handlelength=1.6, handletextpad=.45)
    key.get_frame().set_linewidth(.55)
    return annotations


def place_annotations(ax, annotations, renderer):
    """Keep the mean labels clear of curves, error bars, legends and one another."""
    frame = ax.bbox.padded(-2)
    strokes = [line.get_transform().transform_path(line.get_path()) for line in ax.lines]
    for collection in ax.collections:
        strokes.extend(collection.get_transform().transform_path(MplPath(segment))
                       for segment in collection.get_segments() if len(segment))
    occupied = [ax.get_legend().get_window_extent(renderer).padded(2)]
    offsets = sorted(((dx, dy, side) for dx in (0, -6, 6, -12, 12, -18, 18, -24, 24)
                      for dy in range(4, 29, 2) for side in (1, -1)),
                     key=lambda offset: abs(offset[0]) + offset[1] + (4 if offset[2] < 0 else 0))
    for value, speed, mean, deviation, preferred, color in annotations:
        label = ax.annotate(value, (speed, mean), xytext=(0, 0), textcoords='offset points',
                            ha='center', fontsize=5.6, color=color, zorder=4)
        for dx, dy, side in offsets:
            direction = preferred * side
            label.xy = (speed, mean + direction * deviation)
            label.set_position((dx, dy * direction))
            label.set_va('bottom' if direction > 0 else 'top')
            bounds = label.get_window_extent(renderer).padded(1.2)
            if not (frame.contains(bounds.x0, bounds.y0) and frame.contains(bounds.x1, bounds.y1)):
                continue
            if any(bounds.overlaps(other) for other in occupied):
                continue
            if any(stroke.intersects_bbox(bounds, filled=False) for stroke in strokes):
                continue
            occupied.append(bounds)
            break
        else:
            raise ValueError(f'无法在子图「{ax.get_title()}」内排下均值标注 {value}')


def draw_figure(data):
    fontManager.addfont('/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf')
    mpl.rcParams.update({
        'font.family': ['Liberation Sans', 'Droid Sans Fallback', 'sans-serif'],
        'font.size': 6.5,
        'axes.labelsize': 6.5,
        'axes.spines.top': True,
        'axes.spines.right': True,
        'axes.linewidth': .55,
        'axes.labelpad': 3,
        'axes.unicode_minus': False,
        'xtick.major.width': .55,
        'ytick.major.width': .55,
        'xtick.major.size': 2.6,
        'ytick.major.size': 2.6,
        'xtick.major.pad': 2,
        'ytick.major.pad': 2,
        'pdf.fonttype': 42,
        'svg.fonttype': 'none',
        'savefig.facecolor': 'white',
    })
    fig, axes = plt.subplots(3, 2, figsize=(183 / 25.4, 170 / 25.4))
    fig.subplots_adjust(left=.104, right=.978, bottom=.070, top=.952, wspace=.33, hspace=.51)
    panel_annotations = {}
    for col, (case, route_name) in enumerate(ROUTES):
        groups = [entry for entry in data['summary'] if entry['case'] == case]
        for row, (metric, title, y_label) in enumerate(METRICS):
            ax = axes[row, col]
            panel_annotations[ax] = draw_panel(ax, route_name, groups, metric, title, y_label)
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    for ax, annotations in panel_annotations.items():
        if annotations:
            place_annotations(ax, annotations, renderer)
    return fig


def main():
    source, output = map(Path, sys.argv[1:])
    data = json.loads(source.read_text(encoding='utf-8'))
    fig = draw_figure(data)
    require_matplotlib_panel_alignment(fig, json_out=output / 'plot.alignment.json', strict=True)
    fig.savefig(output / f'{STEM}.pdf')
    fig.savefig(output / f'{STEM}.svg')
    fig.savefig(output / f'{STEM}.png', dpi=300)
    fig.savefig(output / f'{STEM}.tiff', dpi=600, pil_kwargs={'compression': 'tiff_lzw'})
    plt.close(fig)


if __name__ == '__main__':
    main()
