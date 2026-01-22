#!/usr/bin/env python3
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

G = 9.80665

HOOD_MIN_DEG = 0.0
HOOD_MAX_DEG = 85.0

SOLVE_MIN_DEG = 10.0
SOLVE_MAX_DEG = 80.0
SOLVE_SAMPLES = 220

WHEEL_RADIUS_M = 0.0508
WHEEL_SPEED_OVER_MUZZLE = 1.00

DT = 0.01
T_MAX = 3.5


def solve_shot(distance_m: float, target_height_m: float):
    x = max(1e-3, float(distance_m))
    y = float(target_height_m)

    minA = max(math.radians(SOLVE_MIN_DEG), math.radians(HOOD_MIN_DEG))
    maxA = min(math.radians(SOLVE_MAX_DEG), math.radians(HOOD_MAX_DEG))

    best_v = float("inf")
    best_a = minA

    n = max(20, int(SOLVE_SAMPLES))
    for i in range(n + 1):
        a = minA + (maxA - minA) * (i / n)
        c = math.cos(a)
        t = math.tan(a)

        denom_term = x * t - y
        if denom_term <= 1e-6:
            continue

        denom = 2.0 * (c * c) * denom_term
        v2 = (G * x * x) / denom
        if not (v2 > 0.0) or not math.isfinite(v2):
            continue

        v = math.sqrt(v2)
        if v < best_v:
            best_v = v
            best_a = a

    return best_a, best_v


def muzzle_to_shooter_omega(muzzle_mps: float):
    wheel_surface_mps = muzzle_mps * WHEEL_SPEED_OVER_MUZZLE
    if WHEEL_RADIUS_M <= 1e-9:
        return float("inf")
    return wheel_surface_mps / WHEEL_RADIUS_M


def simulate_trajectory(v0: float, angle_rad: float, t_max=T_MAX, dt=DT):
    n = int(t_max / dt) + 1
    t = np.linspace(0.0, t_max, n)
    x = v0 * math.cos(angle_rad) * t
    y = v0 * math.sin(angle_rad) * t - 0.5 * G * t * t
    return x, y


def main():
    fig, ax = plt.subplots()
    plt.subplots_adjust(left=0.10, right=0.98, top=0.94, bottom=0.26)

    ax.set_title("Shooter/Hood Ballistic Solver (No Drag)")
    ax.set_xlabel("Downrange distance (m)")
    ax.set_ylabel("Height (m)")
    ax.grid(True)

    traj_line, = ax.plot([], [])
    target_pt, = ax.plot([], [], marker="o", linestyle="None")
    ground_line, = ax.plot([], [])

    info_text = ax.text(
        0.02, 0.98, "",
        transform=ax.transAxes,
        verticalalignment="top",
        fontsize=10,
    )

    ax_dist = plt.axes([0.12, 0.16, 0.78, 0.03])
    ax_h = plt.axes([0.12, 0.12, 0.78, 0.03])
    ax_tgt = plt.axes([0.12, 0.08, 0.78, 0.03])

    s_dist = Slider(ax_dist, "Distance (m)", 0.5, 12.0, valinit=4.0, valstep=0.01)
    s_h = Slider(ax_h, "Target Height (m)", -1.0, 4.0, valinit=1.5, valstep=0.01)
    s_tgt = Slider(ax_tgt, "Sim Time (s)", 0.5, 6.0, valinit=T_MAX, valstep=0.01)

    ax_btn = plt.axes([0.12, 0.02, 0.12, 0.04])
    btn = Button(ax_btn, "Reset")

    def update(_=None):
        dist = float(s_dist.val)
        h = float(s_h.val)
        tmax = float(s_tgt.val)

        angle, muzzle = solve_shot(dist, h)
        omega = muzzle_to_shooter_omega(muzzle) if math.isfinite(muzzle) else 0.0

        if not math.isfinite(muzzle) or muzzle == float("inf"):
            traj_line.set_data([], [])
            target_pt.set_data([dist], [h])
            info_text.set_text("No valid solution in solve angle range.")
            ax.set_xlim(0.0, max(1.0, dist * 1.1))
            ax.set_ylim(min(-1.0, h - 0.5), max(1.0, h + 1.0))
            fig.canvas.draw_idle()
            return

        x, ytraj = simulate_trajectory(muzzle, angle, t_max=tmax, dt=DT)

        valid = ytraj >= -0.25
        x = x[valid]
        ytraj = ytraj[valid]

        traj_line.set_data(x, ytraj)
        target_pt.set_data([dist], [h])

        gx = np.array([0.0, max(1.0, dist * 1.1)])
        gy = np.array([0.0, 0.0])
        ground_line.set_data(gx, gy)

        ax.set_xlim(0.0, max(1.0, dist * 1.1))
        y_min = min(-0.5, float(np.min(ytraj)) - 0.2) if len(ytraj) else -0.5
        y_max = max(1.0, h + 1.0, float(np.max(ytraj)) + 0.2) if len(ytraj) else max(1.0, h + 1.0)
        ax.set_ylim(y_min, y_max)

        angle_deg = math.degrees(angle)
        info_text.set_text(
            f"Solution:\n"
            f"  Hood angle: {angle_deg:.2f} deg\n"
            f"  Muzzle speed: {muzzle:.2f} m/s\n"
            f"  Shooter ω: {omega:.1f} rad/s\n"
            f"  Shooter RPM: {omega * 60.0 / (2.0 * math.pi):.0f}\n"
            f"Constraints:\n"
            f"  Hood: [{HOOD_MIN_DEG:.1f}, {HOOD_MAX_DEG:.1f}] deg\n"
            f"  Solve angles: [{SOLVE_MIN_DEG:.1f}, {SOLVE_MAX_DEG:.1f}] deg"
        )

        fig.canvas.draw_idle()

    def reset(_):
        s_dist.reset()
        s_h.reset()
        s_tgt.reset()

    s_dist.on_changed(update)
    s_h.on_changed(update)
    s_tgt.on_changed(update)
    btn.on_clicked(reset)

    update()
    plt.show()


if __name__ == "__main__":
    main()
