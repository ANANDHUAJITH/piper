#!/usr/bin/env python3
# plot_trajectory.py
# -------------------------------------------------------
# Plot joint-space, Cartesian-space, and smoothness graphs
# from PIPER arm CSV log files.
#
# Usage:
#   Single file:
#     python3 plot_trajectory.py robot_log_Mode3_YYYYMMDD.csv
#
#   Compare two files (e.g. B-spline vs Quintic):
#     python3 plot_trajectory.py bspline.csv quintic.csv
#
# Output PNG files are saved in the same directory as the CSV.
#
# Requires: pandas, matplotlib
#   pip3 install pandas matplotlib
# -------------------------------------------------------

import sys
import os
import pandas as pd
import matplotlib
matplotlib.use('Agg')   # Non-interactive backend (saves to file)
import matplotlib.pyplot as plt
import numpy as np

JOINT_COLS    = ['Joint1_deg', 'Joint2_deg', 'Joint3_deg',
                 'Joint4_deg', 'Joint5_deg', 'Joint6_deg']
CART_COLS     = ['EndEff_X_mm', 'EndEff_Y_mm', 'EndEff_Z_mm']
GRIPPER_COL   = 'Gripper_mm'
COLORS        = ['#e74c3c', '#2ecc71', '#3498db',
                 '#f39c12', '#9b59b6', '#1abc9c']


def load_csv(filepath):
    """Load a PIPER CSV log. Returns a DataFrame with a time_s column added."""
    df = pd.read_csv(filepath)
    df['Timestamp'] = pd.to_datetime(df['Timestamp'])
    t0 = df['Timestamp'].iloc[0]
    df['time_s'] = (df['Timestamp'] - t0).dt.total_seconds()
    return df


def plot_joint_space(df, out_prefix, label=""):
    """
    Plot all 6 joint angles over time.
    One line per joint, degrees on Y axis, seconds on X axis.
    """
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    axes = axes.flatten()
    fig.suptitle(f'Joint Space — {label}', fontsize=14, fontweight='bold')

    for idx, (col, ax) in enumerate(zip(JOINT_COLS, axes)):
        ax.plot(df['time_s'], df[col], color=COLORS[idx], linewidth=1.5)
        ax.set_title(f'Joint {idx+1}', fontsize=11)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (°)')
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

    plt.tight_layout()
    out = f"{out_prefix}_joint_space.png"
    plt.savefig(out, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {out}")


def plot_cartesian_space(df, out_prefix, label=""):
    """
    Plot end-effector X, Y, Z position over time, plus a top-down XY path.
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Cartesian Space — {label}', fontsize=14, fontweight='bold')

    titles = ['X Position', 'Y Position', 'Z Position', 'XY Path (top view)']
    ylabels = ['X (mm)', 'Y (mm)', 'Z (mm)', 'Y (mm)']

    for i, (col, title, yl) in enumerate(zip(CART_COLS, titles[:3], ylabels[:3])):
        ax = axes.flatten()[i]
        ax.plot(df['time_s'], df[col], color=COLORS[i], linewidth=1.5)
        ax.set_title(title, fontsize=11)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(yl)
        ax.grid(True, alpha=0.3)

    # XY path
    ax_xy = axes[1][1]
    sc = ax_xy.scatter(df['EndEff_X_mm'], df['EndEff_Y_mm'],
                       c=df['time_s'], cmap='viridis', s=10)
    plt.colorbar(sc, ax=ax_xy, label='Time (s)')
    ax_xy.set_title('XY Path (top view)', fontsize=11)
    ax_xy.set_xlabel('X (mm)')
    ax_xy.set_ylabel('Y (mm)')
    ax_xy.grid(True, alpha=0.3)
    ax_xy.set_aspect('equal')

    plt.tight_layout()
    out = f"{out_prefix}_cartesian_space.png"
    plt.savefig(out, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {out}")


def plot_smoothness(df, out_prefix, label=""):
    """
    Plot joint acceleration (second derivative of position over time).
    Lower values = smoother trajectory.
    Uses central differences for numerical differentiation.
    """
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    axes = axes.flatten()
    fig.suptitle(f'Smoothness (Acceleration) — {label}', fontsize=14, fontweight='bold')

    dt = df['time_s'].diff().median()   # Average timestep in seconds

    for idx, (col, ax) in enumerate(zip(JOINT_COLS, axes)):
        vel  = df[col].diff() / dt
        accel = vel.diff() / dt
        ax.plot(df['time_s'], accel, color=COLORS[idx], linewidth=1.0, alpha=0.8)
        ax.axhline(0, color='black', linewidth=0.5)
        ax.set_title(f'Joint {idx+1} Acceleration', fontsize=11)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Accel (°/s²)')
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

    plt.tight_layout()
    out = f"{out_prefix}_smoothness.png"
    plt.savefig(out, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {out}")


def plot_comparison(df1, df2, label1, label2, out_prefix):
    """
    Side-by-side comparison of two trajectories.
    Solid lines = first file, dashed lines = second file.
    """
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    axes = axes.flatten()
    fig.suptitle(f'Comparison: {label1}  vs  {label2}',
                 fontsize=13, fontweight='bold')

    for idx, (col, ax) in enumerate(zip(JOINT_COLS, axes)):
        ax.plot(df1['time_s'], df1[col], color=COLORS[idx],
                linewidth=1.5, label=label1)
        ax.plot(df2['time_s'], df2[col], color=COLORS[idx],
                linewidth=1.5, linestyle='--', label=label2, alpha=0.8)
        ax.set_title(f'Joint {idx+1}', fontsize=11)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (°)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

    plt.tight_layout()
    out = f"{out_prefix}_comparison.png"
    plt.savefig(out, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {out}")


# ----------------------------------------------------------
# Main
# ----------------------------------------------------------
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 plot_trajectory.py <file.csv>")
        print("  python3 plot_trajectory.py <file1.csv> <file2.csv>")
        sys.exit(1)

    file1 = sys.argv[1]
    df1   = load_csv(file1)
    base1 = os.path.splitext(file1)[0]
    label1 = os.path.basename(base1)

    print(f"\nLoaded: {file1}  ({len(df1)} rows)\n")

    # Single file — generate all three graph types
    plot_joint_space(df1,    base1, label=label1)
    plot_cartesian_space(df1, base1, label=label1)
    plot_smoothness(df1,      base1, label=label1)

    # Two files — also generate comparison
    if len(sys.argv) >= 3:
        file2  = sys.argv[2]
        df2    = load_csv(file2)
        base2  = os.path.splitext(file2)[0]
        label2 = os.path.basename(base2)

        print(f"Loaded: {file2}  ({len(df2)} rows)\n")

        # Also plot graphs for file 2
        plot_joint_space(df2,    base2, label=label2)
        plot_cartesian_space(df2, base2, label=label2)
        plot_smoothness(df2,      base2, label=label2)

        # Comparison (save alongside file 1)
        compare_prefix = base1 + "_vs_" + os.path.basename(base2)
        plot_comparison(df1, df2, label1, label2, compare_prefix)

    print("\nAll graphs saved. View with:")
    print("  eog *.png")
    print("  or copy *.png to Desktop")
