#!/usr/bin/env python3
"""Generate an experiment workbook and figures from an existing sweep batch."""

import argparse
import csv
import json
import math
import os
import shutil
import statistics
import subprocess
import sys
from collections import defaultdict
from pathlib import Path
from tempfile import TemporaryDirectory


ANALYSIS_DIR = Path(__file__).resolve().parent
DEPENDENCIES = Path.home() / '.cache/codex-runtimes/codex-primary-runtime/dependencies'
METRICS = ('rmse_m', 'max_error_m', 'mean_speed_kmh', 'steering_rate_rms_deg_s')
FIELDS = ('route', 'controller', 'target_speed_kmh', 'repeat', 'completed', 'outside_lane',
          'elapsed_s', 'endpoint_error_m', *METRICS)
ROUTES = ('forward', 'clockwise')
CONTROLLERS = ('pure_pursuit', 'stanley')


def build_data(batch):
    groups = defaultdict(list)
    identifiers = set()
    with (batch / 'summary.csv').open(newline='') as source:
        reader = csv.DictReader(source)
        if reader.fieldnames != list(FIELDS):
            raise ValueError('summary.csv does not have the expected sweep columns.')
        for line, row in enumerate(reader, 2):
            case, controller = row['route'], row['controller']
            speed, repeat = int(row['target_speed_kmh']), int(row['repeat'])
            key = (case, speed, controller, repeat)
            if case not in ROUTES or controller not in CONTROLLERS or key in identifiers:
                raise ValueError(f'Invalid or duplicate trial on summary.csv line {line}: {key}')
            identifiers.add(key)
            for field, value in row.items():
                if field in ('completed', 'outside_lane'):
                    row[field] = {'True': 1, 'False': 0}[value]
                elif field not in ('route', 'controller'):
                    number = float(value) if value else None
                    if number is not None and not math.isfinite(number):
                        raise ValueError(f'Nonfinite {field} on summary.csv line {line}.')
                    row[field] = number
            groups[case, speed, controller].append(row)

    if not groups:
        raise ValueError('summary.csv contains no trials.')
    rows, summary = [], []
    for (case, speed, controller), group in sorted(
        groups.items(), key=lambda entry: (ROUTES.index(entry[0][0]), entry[0][1], CONTROLLERS.index(entry[0][2]))
    ):
        start = len(rows)
        rows.extend([row[field] for field in FIELDS] for row in sorted(group, key=lambda r: r['repeat']))
        metrics = {}
        for field in METRICS:
            values = [row[field] for row in group if row[field] is not None]
            metrics[field] = [statistics.mean(values) if values else None,
                              statistics.stdev(values) if len(values) > 1 else None]
        summary.append(dict(case=case, speed=speed, controller=controller, start=start, count=len(group),
                            complete=sum(row['completed'] for row in group),
                            outside=sum(row['outside_lane'] for row in group), metrics=metrics))
    return dict(batch=batch.name, fields=FIELDS, rows=rows, summary=summary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path, help='batch directory containing summary.csv')
    parser.add_argument('--output-dir', type=Path, help='defaults to outputs/<batch> beside this script')
    parser.add_argument('--runtime-dir', type=Path, default=DEPENDENCIES,
                        help='bundled workspace dependencies directory containing node/')
    parser.add_argument('--figure-skill', type=Path, default=Path.home() / '.codex/skills/nature-figure',
                        help='nature-figure skill directory for plot alignment checks')
    args = parser.parse_args()

    node = args.runtime_dir / 'node/bin/node'
    modules = args.runtime_dir / 'node/node_modules'
    audit_scripts = args.figure_skill / 'scripts'
    if not node.is_file() or not (modules / '@oai/artifact-tool').is_dir():
        parser.error('Set --runtime-dir to a workspace runtime containing Node.js and @oai/artifact-tool.')
    if not (audit_scripts / 'audit_panel_alignment.py').is_file():
        parser.error('Set --figure-skill to the installed nature-figure skill.')
    data = build_data(args.batch.resolve())
    output = (args.output_dir or (ANALYSIS_DIR / 'outputs' / data['batch'])).resolve()
    output.mkdir(parents=True, exist_ok=True)

    with TemporaryDirectory(prefix='town04-analysis-') as temporary:
        stage = Path(temporary)
        data_path = stage / 'data.json'
        data_path.write_text(json.dumps(data, allow_nan=False), encoding='utf-8')
        workbook_script = stage / 'build_workbook.mjs'
        shutil.copyfile(ANALYSIS_DIR / workbook_script.name, workbook_script)
        (stage / 'node_modules').symlink_to(modules.resolve(), target_is_directory=True)
        env = dict(os.environ, MPLCONFIGDIR=str(stage / 'mpl'))
        env['PYTHONPATH'] = os.pathsep.join(filter(None, (str(audit_scripts.resolve()), env.get('PYTHONPATH'))))
        subprocess.run([sys.executable, '-B', str(ANALYSIS_DIR / 'plot_metrics.py'), str(data_path), str(stage)],
                       env=env, check=True)
        subprocess.run([str(node.resolve()), str(workbook_script), str(data_path), str(stage)], check=True)
        for name in ('Town04_实验结果汇总.xlsx', *(f'Town04_三项指标对比.{ext}' for ext in ('png', 'pdf', 'svg'))):
            shutil.copyfile(stage / name, output / name)
            print(output / name)


if __name__ == '__main__':
    main()
