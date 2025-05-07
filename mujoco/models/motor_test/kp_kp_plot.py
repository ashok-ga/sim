#!/usr/bin/env python3
"""
plot_sweep_results.py

Scans the current directory for `motor_log_kp{kp}_kv{kv}.csv` files,
computes the MSE for each file, identifies the best (lowest MSE) kp/kv pair,
and plots a heatmap of MSE over the Kp/Kv grid.
"""
import os
import re
import csv
import numpy as np
import matplotlib
# Use TkAgg backend for interactive plotting
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


def load_csv(path):
    """Load command and qpos data from CSV for MSE calculation."""
    cmds, qpos = [], []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            cmds.append(float(row['cmd']))
            qpos.append(float(row['qpos']))
    return cmds, qpos


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    # Pattern to extract kp and kv from filenames
    pattern = re.compile(r"motor_log_kp(\d+)_kv(\d+)\.csv")
    data = {}  # map (kp, kv) -> mse

    # Scan files and compute MSE
    for fname in os.listdir(here):
        match = pattern.match(fname)
        if not match:
            continue
        kp, kv = map(int, match.groups())
        path = os.path.join(here, fname)
        cmds, qpos = load_csv(path)
        if cmds and qpos:
            errors = [(c - p)**2 for c, p in zip(cmds, qpos)]
            mse = sum(errors) / len(errors)
        else:
            mse = float('nan')
        data[(kp, kv)] = mse
        print(f"File: {fname} -> kp={kp}, kv={kv}, MSE={mse:.6f}")

    if not data:
        print("No motor_log CSV files found.")
        return

    # Identify best (lowest MSE)
    best_pair, best_mse = min(data.items(), key=lambda item: item[1] if not np.isnan(item[1]) else np.inf)
    best_kp, best_kv = best_pair
    print(f"\nBest pair: kp={best_kp}, kv={best_kv} with MSE={best_mse:.6f}\n")

    # Prepare grid for heatmap
    kp_vals = sorted({kp for kp, _ in data.keys()})
    kv_vals = sorted({kv for _, kv in data.keys()})
    mse_grid = np.full((len(kp_vals), len(kv_vals)), np.nan)
    for (kp, kv), mse in data.items():
        i = kp_vals.index(kp)
        j = kv_vals.index(kv)
        mse_grid[i, j] = mse

    # Plot heatmap
    fig, ax = plt.subplots()
    c = ax.pcolormesh(kv_vals, kp_vals, mse_grid, shading='auto')
    ax.set_xlabel('Kv')
    ax.set_ylabel('Kp')
    ax.set_title('MSE Heatmap for Kp/Kv Sweep')
    fig.colorbar(c, ax=ax, label='MSE (rad^2)')

    # Mark best pair
    ax.plot(best_kv, best_kp, marker='x', color='white', markersize=10, markeredgewidth=2)
    ax.text(best_kv, best_kp,
            f"  Best ({best_kp},{best_kv})",
            color='white', va='center', ha='left', fontsize=8, weight='bold')

    plt.tight_layout()

    # Close on 'q' key press
    def on_key(evt):
        if evt.key == 'q':
            plt.close(fig)
    fig.canvas.mpl_connect('key_press_event', on_key)

    plt.show()

if __name__ == '__main__':
    main()