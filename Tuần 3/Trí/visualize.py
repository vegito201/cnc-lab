"""
visualize.py - Ve quy dao tu output cua C program (trajectory.csv).
Luong: main.exe -> trajectory.csv -> visualize.py -> hinh.

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
    Doc trajectory.csv do main.exe xuat ra.
    Format: t_ms,x,y
    Tra ve list cac (t, x, y).
    """
    points = []
    with open(filepath, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append((float(row['t_ms']), float(row['x']), float(row['y'])))
    return points


# ── Ve hinh ─────────────────────────────────────────
def plot(points, csv_path):
    if not points:
        print("Khong co du lieu de ve.")
        return

    xs = [p[1] for p in points]
    ys = [p[2] for p in points]
    ts = [p[0] for p in points]

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    # ── Left: quy dao XY ──
    ax = axes[0]
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.4)
    ax.set_title('Quy dao XY (tu C interpolation)', fontsize=13)
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')

    # To mau theo thoi gian de phan biet cac doan
    scatter = ax.scatter(xs, ys, c=ts, cmap='plasma', s=4, zorder=3)
    ax.plot(xs, ys, color='royalblue', linewidth=1.2, alpha=0.5, zorder=2)

    # Danh dau start/end
    ax.plot(xs[0], ys[0], 'go', markersize=10, zorder=5, label=f'Start ({xs[0]:.1f}, {ys[0]:.1f})')
    ax.plot(xs[-1], ys[-1], 'r*', markersize=12, zorder=5, label=f'End ({xs[-1]:.1f}, {ys[-1]:.1f})')

    plt.colorbar(scatter, ax=ax, label='Thoi gian (ms)', shrink=0.8)
    ax.legend(loc='upper right', fontsize=9)

    # Padding
    xpad = max((max(xs) - min(xs)) * 0.1, 5)
    ypad = max((max(ys) - min(ys)) * 0.1, 5)
    ax.set_xlim(min(xs) - xpad, max(xs) + xpad)
    ax.set_ylim(min(ys) - ypad, max(ys) + ypad)

    # ── Right: X(t) va Y(t) theo thoi gian ──
    ax2 = axes[1]
    ax2.grid(True, linestyle=':', alpha=0.4)
    ax2.set_title('X(t) va Y(t) theo thoi gian', fontsize=13)
    ax2.set_xlabel('Thoi gian (ms)')
    ax2.set_ylabel('Vi tri (mm)')

    ax2.plot(ts, xs, color='royalblue', linewidth=1.4, label='X(t)')
    ax2.plot(ts, ys, color='crimson',   linewidth=1.4, label='Y(t)')
    ax2.legend()

    plt.tight_layout()

    out = 'trajectory_plot.png'
    plt.savefig(out, dpi=150)
    print(f'Saved: {out}')

    total_pts = len(points)
    total_t   = ts[-1]
    print(f'  {total_pts} diem noi suy | Tong thoi gian: {total_t:.1f} ms ({total_t/1000:.2f} s)')
    print(f'  X: [{min(xs):.2f}, {max(xs):.2f}] mm')
    print(f'  Y: [{min(ys):.2f}, {max(ys):.2f}] mm')


# ── Main ────────────────────────────────────────────
if __name__ == '__main__':
    filepath = sys.argv[1] if len(sys.argv) > 1 else 'trajectory.csv'

    try:
        points = load_csv(filepath)
    except FileNotFoundError:
        print(f'Khong tim thay: {filepath}')
        print('Chay main.exe truoc: ./main.exe test_g9091.gcode')
        sys.exit(1)

    print(f'Doc: {filepath}  ({len(points)} diem)')
    plot(points, filepath)
