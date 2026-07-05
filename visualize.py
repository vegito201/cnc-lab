"""
visualize.py - Ve quy dao tu output cua C program (trajectory.csv).
Luong: main -> trajectory.csv -> visualize.py -> hinh.

Usage:
    python visualize.py [trajectory.csv]
"""

import sys
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── Doc CSV tu C output ──────────────────────────────
def load_csv(filepath: str):
    """
    Doc trajectory.csv do main.c xuat ra.
    Format: t_ms,x,y,v
    Tra ve list cac (t, x, y, v).
    """
    points = []
    with open(filepath, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            v = float(row['v']) if 'v' in row and row['v'] != '' else 0.0
            points.append((float(row['t_ms']), float(row['x']), float(row['y']), v))
    return points


# ── Ve hinh ─────────────────────────────────────────
def plot(points, csv_path):
    if not points:
        print("No data to plot.")
        return

    ts = [p[0] for p in points]
    xs = [p[1] for p in points]
    ys = [p[2] for p in points]
    vs = [p[3] for p in points]

    fig, axes = plt.subplots(1, 3, figsize=(22, 7))

    # ── Subplot 1: quy dao XY ──
    ax = axes[0]
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.4)
    ax.set_title('XY Trajectory (from C interpolation)', fontsize=13)
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')

    scatter = ax.scatter(xs, ys, c=ts, cmap='plasma', s=4, zorder=3)
    ax.plot(xs, ys, color='royalblue', linewidth=1.2, alpha=0.5, zorder=2)

    ax.plot(xs[0], ys[0], 'go', markersize=10, zorder=5,
            label=f'Start ({xs[0]:.1f}, {ys[0]:.1f})')
    ax.plot(xs[-1], ys[-1], 'r*', markersize=12, zorder=5,
            label=f'End ({xs[-1]:.1f}, {ys[-1]:.1f})')

    plt.colorbar(scatter, ax=ax, label='Time (ms)', shrink=0.8)
    ax.legend(loc='upper right', fontsize=9)

    xpad = max((max(xs) - min(xs)) * 0.1, 5)
    ypad = max((max(ys) - min(ys)) * 0.1, 5)
    ax.set_xlim(min(xs) - xpad, max(xs) + xpad)
    ax.set_ylim(min(ys) - ypad, max(ys) + ypad)

    # ── Subplot 2: X(t) va Y(t) ──
    ax2 = axes[1]
    ax2.grid(True, linestyle=':', alpha=0.4)
    ax2.set_title('Position vs Time', fontsize=13)
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('Position (mm)')

    ax2.plot(ts, xs, color='royalblue', linewidth=1.4, label='X(t)')
    ax2.plot(ts, ys, color='crimson',   linewidth=1.4, label='Y(t)')
    ax2.legend()

    # ── Subplot 3: velocity profile v(t) ──
    ax3 = axes[2]
    ax3.grid(True, linestyle=':', alpha=0.4)
    ax3.set_title('Velocity Profile v(t) - Trapezoidal', fontsize=13)
    ax3.set_xlabel('Time (ms)')
    ax3.set_ylabel('Velocity (mm/s)')

    ax3.plot(ts, vs, color='darkorange', linewidth=1.4, label='v(t)')
    ax3.fill_between(ts, vs, alpha=0.15, color='darkorange')

    # danh dau v_max
    v_max = max(vs)
    ax3.axhline(y=v_max, color='gray', linestyle='--', linewidth=0.8, alpha=0.7)
    ax3.text(ts[-1] * 0.02, v_max * 1.02,
             f'v_peak = {v_max:.1f} mm/s', fontsize=9, color='gray')
    ax3.legend()
    ax3.set_ylim(bottom=0)

    plt.tight_layout()

    out = 'trajectory_plot.png'
    plt.savefig(out, dpi=150)
    print(f'Saved: {out}')

    total_pts = len(points)
    total_t   = ts[-1]
    print(f'  {total_pts} points | Total time: {total_t:.1f} ms ({total_t/1000:.2f} s)')
    print(f'  X: [{min(xs):.2f}, {max(xs):.2f}] mm')
    print(f'  Y: [{min(ys):.2f}, {max(ys):.2f}] mm')
    print(f'  v_max = {v_max:.2f} mm/s ({v_max*60:.0f} mm/min)')


# ── Main ────────────────────────────────────────────
if __name__ == '__main__':
    filepath = sys.argv[1] if len(sys.argv) > 1 else 'trajectory.csv'

    try:
        points = load_csv(filepath)
    except FileNotFoundError:
        print(f'File not found: {filepath}')
        print('Run main first: ./main test_g9091.gcode')
        sys.exit(1)

    print(f'Read: {filepath}  ({len(points)} points)')
    plot(points, filepath)
