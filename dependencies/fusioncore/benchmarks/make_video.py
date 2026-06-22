#!/usr/bin/env python3
"""
FusionCore vs robot_localization: trajectory comparison video.

Renders benchmarks/fusioncore_demo_<seq>.mp4 from the TUM trajectory files
already present in benchmarks/nclt/<seq>/.  No ROS, no bag replay required.

Requirements:
    pip install numpy matplotlib opencv-python

Usage (from repo root):
    python3 benchmarks/make_video.py
    python3 benchmarks/make_video.py --seq 2013-04-05
    python3 benchmarks/make_video.py --seq 2012-09-28 --out /tmp/demo.mp4
"""

import argparse
import math
import sys
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

# ── Video parameters ──────────────────────────────────────────────────────────
FPS        = 30
INTRO_S    = 3     # title card
ANIM_S     = 75    # trajectory animation
OUTRO_S    = 12    # result card
TOTAL_S    = INTRO_S + ANIM_S + OUTRO_S   # 90 s
WIDTH      = 1920
HEIGHT     = 1080
RENDER_PTS = 5000  # max trajectory points rendered per frame (speed vs quality)

# ── Colour palette (matches existing benchmark figures) ───────────────────────
BG      = '#0F172A'   # dark slate background
C_MAP   = '#1E293B'   # slightly lighter for the map panel
C_GRID  = '#334155'
C_FC    = '#3B82F6'   # blue  FusionCore
C_EKF   = '#EF4444'   # red   robot_localization EKF
C_GT    = '#94A3B8'   # gray  ground truth
C_GT_DIM= '#334155'   # dimmed full-path preview
C_TEXT  = '#F8FAFC'
C_MUTED = '#94A3B8'
C_GREEN = '#22C55E'

# ── Paper-reported ATE values (SE3 aligned, evo) per sequence ─────────────────
# Used only for the final result card.
PAPER_ATE = {
    '2013-04-05': (12.1,  268.9, 22.2),
    '2012-09-28': (10.8,   55.7,  5.2),
    '2012-12-01': (21.0,   90.7,  4.3),
    '2012-03-31': (22.0,  156.5,  7.1),
    '2012-02-04': (49.7,  265.5,  5.3),
    '2012-01-08': (18.6,   41.2,  2.2),
}


# ── TUM file loader ───────────────────────────────────────────────────────────
def load_tum(path: Path):
    """
    Load a TUM trajectory file.
    Format: timestamp tx ty tz qx qy qz qw  (only first 3 columns used here)
    Returns (timestamps, x, y) as 1-D float64 arrays, NaN/Inf rows excluded.
    """
    rows = []
    with open(path) as fh:
        for line in fh:
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            parts = s.split()
            if len(parts) < 3:
                continue
            try:
                ts, tx, ty = float(parts[0]), float(parts[1]), float(parts[2])
            except ValueError:
                continue
            if any(not math.isfinite(v) for v in (ts, tx, ty)):
                continue
            rows.append((ts, tx, ty))
    if not rows:
        return np.array([]), np.array([]), np.array([])
    a = np.array(rows, dtype=np.float64)
    return a[:, 0], a[:, 1], a[:, 2]


# ── SE(2) alignment (identical to plot_benchmark.py) ─────────────────────────
def align_se2_temporal(src_ts, src_xy, ref_ts, ref_xy):
    """
    Align src trajectory to ref using a temporally matched SE(2) transform.
    Matches each src point to its linearly interpolated ref position, then
    finds the least-squares rotation + translation (no scale).
    """
    step = max(1, len(src_ts) // 2000)
    s_pts, r_pts = [], []
    for i in range(0, len(src_ts), step):
        t = src_ts[i]
        idx = np.searchsorted(ref_ts, t)
        if idx == 0 or idx >= len(ref_ts):
            continue
        t0, t1 = ref_ts[idx - 1], ref_ts[idx]
        if t1 == t0:
            continue
        alpha = (t - t0) / (t1 - t0)
        gx = ref_xy[idx-1, 0] + alpha * (ref_xy[idx, 0] - ref_xy[idx-1, 0])
        gy = ref_xy[idx-1, 1] + alpha * (ref_xy[idx, 1] - ref_xy[idx-1, 1])
        s_pts.append(src_xy[i])
        r_pts.append([gx, gy])
    if len(s_pts) < 3:
        # Not enough correspondences: return untransformed copy
        return src_xy.copy()
    s, r = np.array(s_pts, dtype=np.float64), np.array(r_pts, dtype=np.float64)
    mu_s, mu_r = s.mean(0), r.mean(0)
    H = (s - mu_s).T @ (r - mu_r)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:   # correct reflection
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return (R @ src_xy.T).T + (mu_r - R @ mu_s)


# ── Running ATE (precomputed for efficiency) ──────────────────────────────────
def precompute_running_ate(est_ts, est_xy_aligned, gt_ts, gt_xy):
    """
    Precompute cumulative RMSE of position error at every estimate timestamp.
    Returns an array of the same length as est_ts where entry i is the
    running RMSE of all errors from 0..i.  Values stay NaN until the first
    matched point is found.
    """
    running_sum_sq = 0.0
    running_n      = 0
    result         = np.full(len(est_ts), np.nan, dtype=np.float64)

    for i, t in enumerate(est_ts):
        idx = np.searchsorted(gt_ts, t)
        if idx > 0 and idx < len(gt_ts):
            t0, t1 = gt_ts[idx - 1], gt_ts[idx]
            if t1 != t0:
                alpha = (t - t0) / (t1 - t0)
                gx = gt_xy[idx-1, 0] + alpha * (gt_xy[idx, 0] - gt_xy[idx-1, 0])
                gy = gt_xy[idx-1, 1] + alpha * (gt_xy[idx, 1] - gt_xy[idx-1, 1])
                dx = est_xy_aligned[i, 0] - gx
                dy = est_xy_aligned[i, 1] - gy
                running_sum_sq += dx*dx + dy*dy
                running_n      += 1
        if running_n > 0:
            result[i] = math.sqrt(running_sum_sq / running_n)

    return result


# ── Uniform index subsampling ─────────────────────────────────────────────────
def subsample_idx(n: int, max_pts: int = RENDER_PTS):
    """Return uniformly spaced indices into an array of length n."""
    if n <= max_pts:
        return np.arange(n)
    step = n // max_pts
    return np.arange(0, n, step)


# ── matplotlib figure → OpenCV BGR frame ─────────────────────────────────────
def fig_to_bgr(fig) -> np.ndarray:
    """Render a matplotlib figure into a uint8 BGR numpy array for cv2."""
    fig.canvas.draw()
    buf  = fig.canvas.buffer_rgba()
    rgba = np.frombuffer(buf, dtype=np.uint8).reshape(
        fig.canvas.get_width_height()[::-1] + (4,))
    return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)


# ── Title card ────────────────────────────────────────────────────────────────
def render_title_card() -> np.ndarray:
    fig = plt.figure(figsize=(WIDTH/100, HEIGHT/100), dpi=100, facecolor=BG)
    ax  = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor(BG)
    ax.axis('off')
    kw = dict(transform=ax.transAxes, ha='center', va='center')
    ax.text(0.5, 0.62, 'FusionCore',
            fontsize=72, fontweight='bold', color=C_FC, **kw)
    ax.text(0.5, 0.50, 'vs  robot_localization',
            fontsize=36, color=C_TEXT, **kw)
    ax.text(0.5, 0.38,
            'NCLT Dataset  •  70-min Campus Drive  •  RTK GPS Ground Truth',
            fontsize=18, color=C_MUTED, **kw)
    ax.text(0.5, 0.24, 'github.com/manankharwar/fusioncore',
            fontsize=14, color='#60A5FA', **kw)
    frame = fig_to_bgr(fig)
    plt.close(fig)
    return frame


# ── Result card ───────────────────────────────────────────────────────────────
def render_result_card(seq: str,
                       fc_ate: float, rl_ate: float, ratio: float) -> np.ndarray:
    fig = plt.figure(figsize=(WIDTH/100, HEIGHT/100), dpi=100, facecolor=BG)
    ax  = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor(BG)
    ax.axis('off')
    kw = dict(transform=ax.transAxes, ha='center', va='center')
    ax.text(0.5, 0.82, f'NCLT {seq}  •  Final Result',
            fontsize=20, color=C_MUTED, **kw)
    ax.text(0.5, 0.72, 'Absolute Trajectory Error RMSE',
            fontsize=28, fontweight='bold', color=C_TEXT, **kw)
    # FC
    ax.text(0.30, 0.56, f'{fc_ate:.1f} m',
            fontsize=60, fontweight='bold', color=C_FC, **kw)
    ax.text(0.30, 0.46, 'FusionCore',
            fontsize=20, color=C_FC, **kw)
    # RL
    ax.text(0.70, 0.56, f'{rl_ate:.1f} m',
            fontsize=60, fontweight='bold', color=C_EKF, **kw)
    ax.text(0.70, 0.46, 'robot_localization EKF',
            fontsize=20, color=C_EKF, **kw)
    # Badge
    ax.text(0.5, 0.32, f'{ratio:.1f}× more accurate',
            fontsize=34, fontweight='bold', color=C_GREEN, **kw,
            bbox=dict(boxstyle='round,pad=0.55', facecolor='#14532D',
                      edgecolor=C_GREEN, linewidth=2))
    ax.text(0.5, 0.18,
            'SE(3) aligned  •  evo ATE RMSE  •  single config, no per-sequence tuning',
            fontsize=13, color=C_MUTED, **kw)
    ax.text(0.5, 0.10, 'github.com/manankharwar/fusioncore  •  Apache 2.0',
            fontsize=16, color='#60A5FA', **kw)
    frame = fig_to_bgr(fig)
    plt.close(fig)
    return frame


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='Generate FusionCore trajectory comparison video from TUM files.')
    parser.add_argument('--seq',  default='2013-04-05',
                        choices=list(PAPER_ATE.keys()) + ['custom'],
                        help='NCLT sequence date (default: 2013-04-05)')
    parser.add_argument('--out',  default=None,
                        help='Output MP4 path (default: benchmarks/fusioncore_demo_<seq>.mp4)')
    parser.add_argument('--repo', default=None,
                        help='Repo root (auto-detected from script location)')
    args = parser.parse_args()

    # Resolve paths
    repo    = Path(args.repo) if args.repo else Path(__file__).resolve().parent.parent
    seq_dir = repo / 'benchmarks' / 'nclt' / args.seq
    out_mp4 = Path(args.out) if args.out else \
              repo / 'benchmarks' / f'fusioncore_demo_{args.seq}.mp4'

    for fname in ('fusioncore.tum', 'rl_ekf.tum', 'ground_truth.tum'):
        p = seq_dir / fname
        if not p.exists():
            sys.exit(f'ERROR: required file missing: {p}')

    print(f'Sequence  : {args.seq}')
    print(f'Data dir  : {seq_dir}')
    print(f'Output    : {out_mp4}')

    # Load TUM files
    print('Loading TUM files...')
    gt_ts, gt_x, gt_y = load_tum(seq_dir / 'ground_truth.tum')
    fc_ts, fc_x, fc_y = load_tum(seq_dir / 'fusioncore.tum')
    ek_ts, ek_x, ek_y = load_tum(seq_dir / 'rl_ekf.tum')

    for name, ts in (('ground_truth', gt_ts), ('fusioncore', fc_ts), ('rl_ekf', ek_ts)):
        if len(ts) == 0:
            sys.exit(f'ERROR: {name}.tum loaded zero valid rows')
        dur = (ts[-1] - ts[0]) / 60.0
        print(f'  {name}: {len(ts):,} pts, {dur:.1f} min')

    gt_xy = np.stack([gt_x, gt_y], axis=1)
    fc_xy = np.stack([fc_x, fc_y], axis=1)
    ek_xy = np.stack([ek_x, ek_y], axis=1)

    # SE(2) align both estimates to ground truth
    print('SE(2) alignment...')
    fc_aligned = align_se2_temporal(fc_ts, fc_xy, gt_ts, gt_xy)
    ek_aligned = align_se2_temporal(ek_ts, ek_xy, gt_ts, gt_xy)
    print(f'  FC aligned x=[{fc_aligned[:,0].min():.0f},{fc_aligned[:,0].max():.0f}]'
          f' y=[{fc_aligned[:,1].min():.0f},{fc_aligned[:,1].max():.0f}]')
    print(f'  EKF aligned x=[{ek_aligned[:,0].min():.0f},{ek_aligned[:,0].max():.0f}]'
          f' y=[{ek_aligned[:,1].min():.0f},{ek_aligned[:,1].max():.0f}]')

    # Precompute running ATE for both
    print('Precomputing running ATE...')
    fc_cumrmse = precompute_running_ate(fc_ts, fc_aligned, gt_ts, gt_xy)
    ek_cumrmse = precompute_running_ate(ek_ts, ek_aligned, gt_ts, gt_xy)
    fc_final_se2 = float(np.nanmax(fc_cumrmse))
    ek_final_se2 = float(np.nanmax(ek_cumrmse))
    print(f'  FC final SE2 running RMSE: {fc_final_se2:.1f} m')
    print(f'  EKF final SE2 running RMSE: {ek_final_se2:.1f} m')

    # Subsample for rendering efficiency
    fc_idx  = subsample_idx(len(fc_ts))
    ek_idx  = subsample_idx(len(ek_ts))
    fc_ts_r = fc_ts[fc_idx];       fc_al_r = fc_aligned[fc_idx]
    ek_ts_r = ek_ts[ek_idx];       ek_al_r = ek_aligned[ek_idx]
    fc_rm_r = fc_cumrmse[fc_idx]
    ek_rm_r = ek_cumrmse[ek_idx]

    # Map viewport: ground truth bounding box + padding
    PAD = 80  # meters
    xlo = gt_x.min() - PAD;   xhi = gt_x.max() + PAD
    ylo = gt_y.min() - PAD;   yhi = gt_y.max() + PAD
    print(f'Map viewport: x=[{xlo:.0f},{xhi:.0f}] y=[{ylo:.0f},{yhi:.0f}]')

    # Time span covering all three trajectories
    t_start = min(gt_ts[0],  fc_ts[0],  ek_ts[0])
    t_end   = max(gt_ts[-1], fc_ts[-1], ek_ts[-1])
    total_data_s = t_end - t_start
    speed_factor = total_data_s / ANIM_S
    print(f'Data span: {total_data_s/60:.1f} min  ({speed_factor:.0f}x playback speed)')

    # Paper ATE values for result card
    if args.seq in PAPER_ATE:
        fc_ate_p, rl_ate_p, ratio_p = PAPER_ATE[args.seq]
    else:
        fc_ate_p  = fc_final_se2
        rl_ate_p  = ek_final_se2
        ratio_p   = rl_ate_p / fc_ate_p if fc_ate_p > 0 else 1.0

    # Legend (constant across all animation frames)
    legend_elems = [
        Line2D([0], [0], color=C_GT,  lw=2.0, ls='--', label='Ground Truth (RTK GPS)'),
        Line2D([0], [0], color=C_FC,  lw=2.5,           label='FusionCore UKF'),
        Line2D([0], [0], color=C_EKF, lw=2.0,           label='robot_localization EKF'),
    ]

    # Open video writer
    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(str(out_mp4), fourcc, FPS, (WIDTH, HEIGHT))
    if not writer.isOpened():
        sys.exit(f'ERROR: cv2.VideoWriter could not open {out_mp4}')

    total_frames = TOTAL_S * FPS
    anim_frames  = ANIM_S  * FPS
    frames_written = 0
    print(f'\nRendering {total_frames} frames ({TOTAL_S}s @ {FPS}fps)...')

    # Section 1: intro title card
    print('  [1/3] Title card...')
    title_frame = render_title_card()
    for _ in range(INTRO_S * FPS):
        writer.write(title_frame)
        frames_written += 1

    # Section 2: trajectory animation
    print('  [2/3] Trajectory animation...')
    for anim_i in range(anim_frames):
        progress = anim_i / max(anim_frames - 1, 1)   # 0.0 to 1.0
        t_now    = t_start + progress * total_data_s

        # Number of subsampled render points visible so far
        fc_n = int(np.searchsorted(fc_ts_r, t_now, side='right'))
        ek_n = int(np.searchsorted(ek_ts_r, t_now, side='right'))
        gt_n = int(np.searchsorted(gt_ts,   t_now, side='right'))

        # Running ATE at this moment (last valid value in the running array)
        def latest_rmse(rm_arr, n):
            if n == 0:
                return 0.0
            # search backward for last non-nan
            for j in range(n - 1, -1, -1):
                v = rm_arr[j]
                if math.isfinite(v):
                    return v
            return 0.0

        fc_live = latest_rmse(fc_rm_r, fc_n)
        ek_live = latest_rmse(ek_rm_r, ek_n)

        # Elapsed time string (data time, not wall time)
        elapsed_data_s  = int(progress * total_data_s)
        elapsed_m, elapsed_s2 = divmod(elapsed_data_s, 60)
        total_m = int(total_data_s / 60)
        elapsed_str = f'{elapsed_m:02d}:{elapsed_s2:02d} / {total_m:02d}:00'

        # ── Build frame ───────────────────────────────────────────────────────
        fig = plt.figure(figsize=(WIDTH/100, HEIGHT/100), dpi=100, facecolor=BG)

        # Map axis (left 64% of figure width)
        ax = fig.add_axes([0.02, 0.09, 0.62, 0.85])
        ax.set_facecolor(C_MAP)
        for sp in ax.spines.values():
            sp.set_edgecolor(C_GRID)
        ax.tick_params(colors=C_MUTED, labelsize=9)
        ax.set_xlabel('East (m)',  fontsize=10, color=C_MUTED, labelpad=3)
        ax.set_ylabel('North (m)', fontsize=10, color=C_MUTED, labelpad=3)
        ax.set_xlim(xlo, xhi)
        ax.set_ylim(ylo, yhi)
        ax.set_aspect('equal')
        ax.grid(color=C_GRID, lw=0.5, zorder=0)

        # Full ground truth path (dim preview so viewer knows the route)
        ax.plot(gt_x, gt_y, color=C_GT_DIM, lw=1.0, ls='--', zorder=1, alpha=0.5)

        # Ground truth drawn so far (bright)
        if gt_n > 1:
            ax.plot(gt_x[:gt_n], gt_y[:gt_n],
                    color=C_GT, lw=1.8, ls='--', zorder=2, alpha=0.9)

        # RL-EKF drawn so far
        if ek_n > 1:
            # Clip to a generous window so wild divergences stay visible
            # but don't crash the clipping math on moderate overruns
            ek_clip_x = np.clip(ek_al_r[:ek_n, 0], xlo - 600, xhi + 600)
            ek_clip_y = np.clip(ek_al_r[:ek_n, 1], ylo - 600, yhi + 600)
            ax.plot(ek_clip_x, ek_clip_y,
                    color=C_EKF, lw=2.0, zorder=3, alpha=0.85)

        # FusionCore drawn so far
        if fc_n > 1:
            ax.plot(fc_al_r[:fc_n, 0], fc_al_r[:fc_n, 1],
                    color=C_FC, lw=2.5, zorder=4)

        # Start marker
        ax.plot(gt_x[0], gt_y[0], 'o', color=C_TEXT, ms=7, zorder=6)
        ax.text(gt_x[0] + 15, gt_y[0] + 8, 'START',
                color=C_TEXT, fontsize=9, va='bottom')

        # Legend
        leg = ax.legend(handles=legend_elems, loc='upper right',
                        fontsize=9, facecolor=C_MAP,
                        edgecolor=C_GRID, framealpha=0.92)
        for lt in leg.get_texts():
            lt.set_color(C_TEXT)

        # ── Right panel (text overlays, x=0.665 to 0.985) ────────────────────
        rx = 0.665   # left edge of right panel in figure coords

        # Panel heading
        fig.text(rx, 0.94, f'NCLT {args.seq}',
                 fontsize=15, fontweight='bold', color=C_MUTED)
        fig.text(rx, 0.905, f'{total_m}-min campus drive  •  RTK ground truth',
                 fontsize=10, color=C_MUTED)

        # FusionCore live ATE
        fig.text(rx, 0.845, 'FusionCore UKF',
                 fontsize=12, fontweight='bold', color=C_FC)
        fig.text(rx, 0.775, f'{fc_live:.1f} m',
                 fontsize=48, fontweight='bold', color=C_FC)
        fig.text(rx, 0.745, 'running ATE RMSE (SE2 aligned)',
                 fontsize=9, color=C_MUTED)

        # RL-EKF live ATE
        fig.text(rx, 0.685, 'robot_localization EKF',
                 fontsize=12, fontweight='bold', color=C_EKF)
        fig.text(rx, 0.615, f'{ek_live:.1f} m',
                 fontsize=48, fontweight='bold', color=C_EKF)
        fig.text(rx, 0.585, 'running ATE RMSE (SE2 aligned)',
                 fontsize=9, color=C_MUTED)

        # Improvement ratio (shown after 10% of animation to let values stabilise)
        if progress > 0.10 and fc_live > 0.5:
            ratio_live = ek_live / fc_live
            fig.text(rx, 0.535, f'{ratio_live:.1f}×  more accurate',
                     fontsize=18, fontweight='bold', color=C_GREEN)

        # Progress bar background
        bar_axes_bg = fig.add_axes([rx, 0.49, 0.31, 0.014])
        bar_axes_bg.set_facecolor(C_GRID)
        bar_axes_bg.axis('off')
        # Progress bar fill
        if progress > 0:
            bar_axes_fg = fig.add_axes([rx, 0.49, 0.31 * progress, 0.014])
            bar_axes_fg.set_facecolor(C_FC)
            bar_axes_fg.axis('off')

        fig.text(rx, 0.468, f'Elapsed: {elapsed_str}',
                 fontsize=10, color=C_MUTED)

        # Top title bar
        fig.text(0.5, 0.977,
                 'FusionCore vs robot_localization  •  Sensor Fusion Benchmark',
                 ha='center', fontsize=13, color=C_MUTED)

        # Write frame to video
        bgr = fig_to_bgr(fig)
        writer.write(bgr)
        plt.close(fig)
        frames_written += 1

        if (anim_i + 1) % 150 == 0 or anim_i == anim_frames - 1:
            pct = (anim_i + 1) / anim_frames * 100
            print(f'    {anim_i+1:4d}/{anim_frames}  ({pct:.0f}%)  '
                  f'FC={fc_live:.1f}m  EKF={ek_live:.1f}m')

    # Section 3: result card
    print('  [3/3] Result card...')
    result_frame = render_result_card(args.seq, fc_ate_p, rl_ate_p, ratio_p)
    for _ in range(OUTRO_S * FPS):
        writer.write(result_frame)
        frames_written += 1

    writer.release()
    size_mb = out_mp4.stat().st_size / 1e6
    print(f'\nDone.  {frames_written} frames  ->  {out_mp4}  ({size_mb:.1f} MB)')


if __name__ == '__main__':
    main()
