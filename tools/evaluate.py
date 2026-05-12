#!/usr/bin/env python3
"""
NCLT benchmark evaluator: FusionCore vs robot_localization.

Metrics computed per filter (SE3-aligned to ground truth):
  - ATE translational RMSE, mean, max
  - ATE rotational RMSE (degrees)
  - % of poses within 5 m and 10 m of ground truth
  - Path length ratio (estimated / ground truth)
  - Drift rate (ATE RMSE per km traveled)
  - RPE at 10 m, 50 m, 100 m segment lengths

Prerequisites:
  pip install evo matplotlib

Usage:
  python3 tools/evaluate.py \
    --gt           ground_truth.tum \
    --fusioncore   fusioncore.tum \
    --rl           rl_ekf.tum \
    --sequence     2012-01-08 \
    --out_dir      ./benchmarks/nclt/2012-01-08/results
"""

import argparse
import copy
import os
import sys
from pathlib import Path

import numpy as np

try:
    from evo.core import metrics, sync
    from evo.core.trajectory import PoseTrajectory3D
    from evo.tools import file_interface
except ImportError:
    print("Error: evo not installed. Run: pip install evo", file=sys.stderr)
    sys.exit(1)


# ── loaders ────────────────────────────────────────────────────────────────────

def load_tum(path: str) -> PoseTrajectory3D:
    return file_interface.read_tum_trajectory_file(path)


def align(est: PoseTrajectory3D, gt: PoseTrajectory3D) -> PoseTrajectory3D:
    """Return SE3-aligned copy of est matched to gt timestamps."""
    est_copy = copy.deepcopy(est)
    est_copy.align(gt, correct_scale=False)
    return est_copy


# ── metric helpers ─────────────────────────────────────────────────────────────

def compute_ate(gt: PoseTrajectory3D, est: PoseTrajectory3D) -> dict:
    gt_s, est_s = sync.associate_trajectories(gt, est, max_diff=0.1)
    est_aligned = align(est_s, gt_s)
    metric = metrics.APE(metrics.PoseRelation.translation_part)
    metric.process_data((gt_s, est_aligned))
    errors = metric.error
    return {
        'rmse':   float(np.sqrt(np.mean(errors**2))),
        'mean':   float(np.mean(errors)),
        'max':    float(np.max(errors)),
        'pct5':   float(np.mean(errors <= 5.0) * 100),
        'pct10':  float(np.mean(errors <= 10.0) * 100),
        'errors': errors,
        'gt_s':   gt_s,
        'est_aligned': est_aligned,
    }


def compute_ate_rot(gt: PoseTrajectory3D, est: PoseTrajectory3D) -> float:
    gt_s, est_s = sync.associate_trajectories(gt, est, max_diff=0.1)
    est_aligned = align(est_s, gt_s)
    metric = metrics.APE(metrics.PoseRelation.rotation_angle_deg)
    metric.process_data((gt_s, est_aligned))
    return float(np.sqrt(np.mean(metric.error**2)))


def compute_rpe(gt: PoseTrajectory3D, est: PoseTrajectory3D, delta_m: float) -> dict:
    gt_s, est_s = sync.associate_trajectories(gt, est, max_diff=0.1)
    metric = metrics.RPE(
        metrics.PoseRelation.translation_part,
        delta=delta_m,
        delta_unit=metrics.Unit.meters,
        all_pairs=False,
    )
    try:
        metric.process_data((gt_s, est_s))
        errors = metric.error
        return {'rmse': float(np.sqrt(np.mean(errors**2))),
                'mean': float(np.mean(errors)),
                'max':  float(np.max(errors))}
    except Exception:
        return {'rmse': float('nan'), 'mean': float('nan'), 'max': float('nan')}


def path_length(traj: PoseTrajectory3D) -> float:
    diffs = np.diff(traj.positions_xyz, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def path_length_ratio(gt: PoseTrajectory3D, est: PoseTrajectory3D) -> float:
    gt_s, est_s = sync.associate_trajectories(gt, est, max_diff=0.1)
    gt_len  = path_length(gt_s)
    est_len = path_length(est_s)
    return est_len / gt_len if gt_len > 0 else float('nan')


def drift_rate(ate_rmse: float, gt: PoseTrajectory3D) -> float:
    km = path_length(gt) / 1000.0
    return ate_rmse / km if km > 0 else float('nan')


# ── printing ───────────────────────────────────────────────────────────────────

def fmt(val, unit='m', decimals=3):
    if val != val:
        return 'N/A'
    return f'{val:.{decimals}f}{unit}'


def print_section(title: str):
    pad = max(0, 54 - len(title))
    print(f'\n── {title} {"─" * pad}')


def evaluate_filter(label: str, gt: PoseTrajectory3D, est: PoseTrajectory3D) -> dict:
    ate   = compute_ate(gt, est)
    rpe10 = compute_rpe(gt, est, 10.0)
    plr   = path_length_ratio(gt, est)
    dr    = drift_rate(ate['rmse'], gt)

    print(f'\n  [{label}]')
    print(f'    ATE RMSE            {fmt(ate["rmse"])}')
    print(f'    Within 5 m          {ate["pct5"]:.1f}%  |  Within 10 m  {ate["pct10"]:.1f}%')
    print(f'    Path length ratio   {fmt(plr, "", 4)}  (1.0 = perfect scale)')
    print(f'    Drift rate          {fmt(dr, " m/km", 2)}')
    print(f'    RPE@10m             RMSE={fmt(rpe10["rmse"])}')

    return {'ate': ate, 'rpe10': rpe10, 'plr': plr, 'drift': dr}


# ── plots ──────────────────────────────────────────────────────────────────────

def save_trajectory_plot(gt, fc_res, rl_res, out_dir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(10, 8))

        gt_xy  = gt.positions_xyz[:, :2]
        fc_xy  = fc_res['ate']['est_aligned'].positions_xyz[:, :2]
        rl_xy  = rl_res['ate']['est_aligned'].positions_xyz[:, :2]

        ax.plot(gt_xy[:, 0], gt_xy[:, 1], '--', color='#2ca02c', label='Ground Truth', alpha=0.7, lw=1.5)
        ax.plot(fc_xy[:, 0], fc_xy[:, 1], '-',  color='#1f77b4', label='FusionCore',   alpha=0.85, lw=1.2)
        ax.plot(rl_xy[:, 0], rl_xy[:, 1], '-',  color='#ff7f0e', label='RL-EKF',       alpha=0.85, lw=1.2)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Trajectory Overlay (XY, SE3-aligned)')
        ax.set_aspect('equal')
        ax.legend()
        ax.grid(True, alpha=0.3)

        path = os.path.join(out_dir, 'trajectories.png')
        fig.savefig(path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f'  Trajectory overlay  → {path}')
    except Exception as e:
        print(f'  [WARN] trajectory plot failed: {e}')


def save_ate_plot(gt, fc_res, rl_res, out_dir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(12, 4))

        for label, res, color in [('FusionCore', fc_res, '#1f77b4'),
                                   ('RL-EKF',     rl_res, '#ff7f0e')]:
            t = res['ate']['gt_s'].timestamps
            t = t - t[0]
            ax.plot(t, res['ate']['errors'], label=f'{label} (RMSE={res["ate"]["rmse"]:.1f}m)',
                    color=color, alpha=0.8, lw=0.8)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('ATE (m)')
        ax.set_title('Per-pose Translational Error over Time')
        ax.legend()
        ax.grid(True, alpha=0.3)

        path = os.path.join(out_dir, 'ate_over_time.png')
        fig.savefig(path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f'  ATE over time       → {path}')
    except Exception as e:
        print(f'  [WARN] ATE over time plot failed: {e}')


def save_error_distribution(fc_res, rl_res, out_dir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(8, 5))
        thresholds = np.linspace(0, 50, 200)

        for label, res, color in [('FusionCore', fc_res, '#1f77b4'),
                                   ('RL-EKF',     rl_res, '#ff7f0e')]:
            errors = res['ate']['errors']
            cdf = [np.mean(errors <= t) * 100 for t in thresholds]
            ax.plot(thresholds, cdf, label=label, color=color, lw=2)

        ax.axvline(5,  color='gray', linestyle=':', alpha=0.7, label='5 m')
        ax.axvline(10, color='gray', linestyle='--', alpha=0.7, label='10 m')
        ax.set_xlabel('ATE threshold (m)')
        ax.set_ylabel('% poses within threshold')
        ax.set_title('Error Distribution (CDF)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, 50)
        ax.set_ylim(0, 100)

        path = os.path.join(out_dir, 'error_distribution.png')
        fig.savefig(path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f'  Error distribution  → {path}')
    except Exception as e:
        print(f'  [WARN] error distribution plot failed: {e}')


# ── markdown ───────────────────────────────────────────────────────────────────

def write_markdown(fc: dict, rl: dict, sequence: str, out_dir: str):
    md_path = os.path.join(out_dir, 'BENCHMARK.md')
    with open(md_path, 'w') as f:
        f.write(f'# Benchmark Results: NCLT Sequence {sequence}\n\n')

        f.write('## Metrics (SE3-aligned to RTK ground truth)\n\n')
        f.write('| Filter | ATE RMSE (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |\n')
        f.write('|--------|-------------|------------|-------------|-------------------|--------------|------------------|\n')
        for label, r in [('FusionCore', fc), ('RL-EKF', rl)]:
            ate = r['ate']
            f.write(f"| {label} | {ate['rmse']:.3f} | {ate['pct5']:.1f}% | {ate['pct10']:.1f}% | "
                    f"{r['plr']:.4f} | {r['drift']:.2f} | {r['rpe10']['rmse']:.3f} |\n")

        f.write('\n## Methodology\n\n')
        f.write('- Dataset: NCLT (University of Michigan)\n')
        f.write(f'- Sequence: {sequence}\n')
        f.write('- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU\n')
        f.write('- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment\n')
        f.write('- Motion model: DifferentialDrive\n')
        f.write('- Sensor inputs: identical for all filters (IMU + wheel odom + GPS)\n')

    print(f'  Results written to {md_path}')


# ── main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--gt',         required=True)
    parser.add_argument('--fusioncore', required=True)
    parser.add_argument('--rl',         required=True)
    parser.add_argument('--sequence',   default='unknown')
    parser.add_argument('--out_dir',    default='./results')
    args = parser.parse_args()

    for path, name in [(args.gt, 'ground truth'), (args.fusioncore, 'FusionCore'),
                       (args.rl, 'robot_localization')]:
        if not os.path.exists(path):
            print(f'Error: {name} file not found: {path}', file=sys.stderr)
            sys.exit(1)

    Path(args.out_dir).mkdir(parents=True, exist_ok=True)

    gt = load_tum(args.gt)
    fc = load_tum(args.fusioncore)
    rl = load_tum(args.rl)

    print(f'\n{"="*60}')
    print(f'  NCLT Benchmark: {args.sequence}')
    print(f'{"="*60}')
    print(f'  Poses: FC={len(fc.timestamps)}  RL={len(rl.timestamps)}  GT={len(gt.timestamps)}')

    print_section('Metrics (SE3-aligned to RTK ground truth)')
    fc_res = evaluate_filter('FusionCore', gt, fc)
    rl_res = evaluate_filter('RL-EKF',     gt, rl)

    print_section('Summary')
    fc_ate = fc_res['ate']['rmse']
    rl_ate = rl_res['ate']['rmse']
    if fc_ate < rl_ate:
        diff = (rl_ate - fc_ate) / rl_ate * 100
        print(f'  FusionCore wins ATE by {diff:.1f}%  ({fc_ate:.3f}m vs {rl_ate:.3f}m)')
    elif rl_ate < fc_ate:
        diff = (fc_ate - rl_ate) / fc_ate * 100
        print(f'  RL-EKF wins ATE by {diff:.1f}%  ({rl_ate:.3f}m vs {fc_ate:.3f}m)')
    else:
        print(f'  Tie  ({fc_ate:.3f}m)')

    print_section('Plots')
    save_trajectory_plot(gt, fc_res, rl_res, args.out_dir)
    save_ate_plot(gt, fc_res, rl_res, args.out_dir)
    save_error_distribution(fc_res, rl_res, args.out_dir)

    write_markdown(fc_res, rl_res, args.sequence, args.out_dir)
    print(f'\nDone. Results in {args.out_dir}/\n')


if __name__ == '__main__':
    main()
