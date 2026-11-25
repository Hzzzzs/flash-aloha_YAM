"""Offline same-tick tracking statistics for a single dual-YAM replay segment.

Joint units are degrees; gripper errors are percentage points of normalized opening.
This is command-versus-immediately-read feedback, not settled accuracy or zero error.
No temporal shift, resampling, angle wrapping or hardware access is performed.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def summarize(target, qpos, times, rows):
    target, qpos = np.asarray(target), np.asarray(qpos)
    times, rows = np.asarray(times), np.asarray(rows)
    if target.ndim != 2 or target.shape[1] != 14 or qpos.shape != target.shape or not len(target):
        raise ValueError('Expected nonempty matching (N, 14) target and qpos')
    if times.shape != (len(target),) or rows.shape != times.shape:
        raise ValueError('time and row must have one entry per sample')
    if not all(np.isfinite(x).all() for x in (target, qpos, times, rows)):
        raise ValueError('Replay contains non-finite values')
    if np.any(np.diff(times) <= 0):
        raise ValueError('Replay wall-clock timestamps must increase within the segment')
    if np.any(rows != np.floor(rows)) or np.any(rows < 0) or np.any(np.diff(rows) != 1):
        raise ValueError('Replay rows must be consecutive nonnegative integers')
    stats = []
    for column in range(14):
        gripper = column % 7 == 6
        error = (qpos[:, column] - target[:, column]) * (100. if gripper else 180. / np.pi)
        absolute = np.abs(error)
        peak = int(np.argmax(absolute))
        stats.append(dict(arm='left' if column < 7 else 'right',
            joint='gripper' if gripper else f'J{column % 7 + 1}',
            unit='percentage_points' if gripper else 'degrees',
            bias=float(error.mean()), mae=float(absolute.mean()),
            rmse=float(np.sqrt(np.mean(error ** 2))), p95_abs=float(np.percentile(absolute, 95)),
            max_abs=float(absolute[peak]), peak_row=int(rows[peak]),
            peak_elapsed_seconds=float(times[peak] - times[0])))
    duration = float(times[-1] - times[0])
    return dict(schema_version=1, samples=len(target), first_row=int(rows[0]), last_row=int(rows[-1]),
        elapsed_seconds=duration, measured_log_hz=(len(target)-1)/duration if duration else None,
        measurement='same-tick commanded target versus feedback read immediately after send; no time shift',
        error_sign='qpos - target', metrics=stats)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('segment', type=Path, help='One steps_*.npz file; analyze segments separately')
    parser.add_argument('--output', type=Path, help='Optional JSON report; otherwise prints only')
    parser.add_argument('--plot', type=Path, help='Optional J2/J3 plot (requires matplotlib)')
    args = parser.parse_args()
    with np.load(args.segment, allow_pickle=False) as data:
        report = summarize(data['target'], data['qpos'], data['time'], data['row'])
        if args.plot:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            figure, axes = plt.subplots(2, 2, figsize=(12, 6.8), sharex=True, layout='constrained')
            seconds = data['time'] - data['time'][0]
            for side, arm in enumerate(('Left', 'Right')):
                for row, joint in enumerate((1, 2)):
                    column = side * 7 + joint
                    axis = axes[row, side]
                    axis.plot(seconds, np.rad2deg(data['target'][:, column]), label='Target', linewidth=1.3)
                    axis.plot(seconds, np.rad2deg(data['qpos'][:, column]), label='Feedback', linewidth=1.1)
                    axis.set_title(f'{arm} J{joint + 1}')
                    axis.set_ylabel('Angle (degrees)')
                    axis.grid(alpha=.25)
                    if row == 1:
                        axis.set_xlabel('Replay elapsed time (seconds)')
            axes[0, 0].legend()
            figure.suptitle('YAM replay: target and immediate feedback\nNo time shift; dynamic tracking, not settled accuracy')
            args.plot.parent.mkdir(parents=True, exist_ok=True)
            figure.savefig(args.plot, dpi=150)
            plt.close(figure)
    report['source_segment'] = args.segment.name
    run_path = args.segment.parent / 'run.json'
    if run_path.exists():
        run = json.loads(run_path.read_text())
        report['expected_rows'] = run.get('rows')
        report['all_expected_rows_logged'] = bool(report['first_row'] == 0
            and report['samples'] == run.get('rows'))
        report['nominal_hz'] = run.get('fps')
    text = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    print(text, end='')


if __name__ == '__main__':
    main()
