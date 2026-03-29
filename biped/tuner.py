"""interactive parameter tuner with live gait visualization"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.animation import FuncAnimation

from .gait import GaitGenerator, GaitCommand
from .kinematics import forward_kinematics
from .servo import ServoMapper
from .config import save_config


def run_tuner(cfg: dict):
    """opens the tuning gui with sliders and a live leg animation"""
    rp = dict(cfg["robot"])
    gp = dict(cfg["gait"])

    gait = GaitGenerator(rp, gp)
    mapper = ServoMapper(cfg.get("servo_calibration"))

    walking = [False]
    cmd = [GaitCommand()]

    fig = plt.figure(figsize=(14, 9))
    fig.suptitle("Biped Gait Tuner — adjust sliders, press Walk/Stand", fontsize=13)

    ax_side = fig.add_axes([0.05, 0.45, 0.4, 0.5])
    ax_angles = fig.add_axes([0.52, 0.45, 0.44, 0.5])

    ax_side.set_aspect("equal")
    ax_side.set_title("Side view", fontsize=11)
    ax_side.grid(True, alpha=0.3)
    ground_line = ax_side.axhline(y=0, color="saddlebrown", lw=2, zorder=1)
    ax_side.axhspan(-0.02, 0, color="saddlebrown", alpha=0.15, zorder=0)

    ax_angles.set_title("Left leg angles", fontsize=11)
    ax_angles.grid(True, alpha=0.3)
    ax_angles.set_ylim(-90, 130)
    ax_angles.set_ylabel("degrees")

    angle_hist = {"hip": [], "knee": [], "ankle": []}
    max_hist = 150

    left_line, = ax_side.plot([], [], "b-o", lw=2, ms=5, label="Left")
    right_line, = ax_side.plot([], [], "r-o", lw=2, ms=5, label="Right")
    torso_dot, = ax_side.plot([], [], "ks", ms=10)
    ax_side.legend(fontsize=8, loc="upper right")

    hip_line, = ax_angles.plot([], [], "b-", lw=1.5, label="hip pitch")
    knee_line, = ax_angles.plot([], [], "g-", lw=1.5, label="knee pitch")
    ankle_line, = ax_angles.plot([], [], color="orange", lw=1.5, label="ankle pitch")
    ax_angles.legend(fontsize=8, loc="upper right")

    slider_specs = [
        ("stand_height", "Stand H (cm)", 6, 20, 100),
        ("step_height", "Step H (cm)", 0.5, 6, 100),
        ("step_length", "Step L (cm)", 1, 8, 100),
        ("step_period", "Period (s)", 0.4, 3.0, 1),
        ("lateral_sway", "Sway (cm)", 0.0, 5, 100),
        ("duty_factor", "Duty", 0.5, 0.85, 1),
    ]

    sliders = {}
    for i, (key, label, vmin, vmax, scale) in enumerate(slider_specs):
        row = i // 2
        col = i % 2
        ax_s = fig.add_axes([0.08 + col * 0.48, 0.32 - row * 0.07, 0.38, 0.03])
        src = gp if key in gp else rp
        init_val = src[key] * scale
        s = Slider(ax_s, label, vmin, vmax, valinit=init_val, valstep=0.5 if scale > 1 else 0.05)
        sliders[key] = (s, scale, key in gp)

    def sync_params():
        for key, (s, scale, is_gait) in sliders.items():
            val = s.val / scale
            if is_gait:
                gp[key] = val
            else:
                rp[key] = val
        gait.update_params(rp, gp)

    for _, (s, _, _) in sliders.items():
        s.on_changed(lambda val: sync_params())

    ax_walk = fig.add_axes([0.08, 0.02, 0.1, 0.04])
    ax_stand = fig.add_axes([0.20, 0.02, 0.1, 0.04])
    ax_save = fig.add_axes([0.32, 0.02, 0.14, 0.04])

    btn_walk = Button(ax_walk, "Walk")
    btn_stand = Button(ax_stand, "Stand")
    btn_save = Button(ax_save, "Save config")

    def on_walk(event):
        walking[0] = True
        cmd[0] = GaitCommand(forward_vel=0.03)

    def on_stand(event):
        walking[0] = False
        cmd[0] = GaitCommand()
        gait.reset()

    def on_save(event):
        cfg["robot"] = dict(rp)
        cfg["gait"] = dict(gp)
        save_config(cfg, "configs/tuned.yaml")

    btn_walk.on_clicked(on_walk)
    btn_stand.on_clicked(on_stand)
    btn_save.on_clicked(on_save)

    def animate(frame):
        dt = 1.0 / 30.0

        if walking[0]:
            state = gait.update(dt, cmd[0])
        else:
            state = gait.update(dt, GaitCommand())

        stand_h = gp["stand_height"]
        sway = gp["lateral_sway"] * np.sin(2 * np.pi * (gait.phase + 0.1)) if walking[0] else 0

        lpts = forward_kinematics((0, stand_h), state.left, rp)
        rpts = forward_kinematics((0, stand_h), state.right, rp)

        max_h = stand_h * 1.3

        ax_side.set_xlim(-0.12, 0.12)
        ax_side.set_ylim(-0.005, max_h)

        lx = [p[0] for p in lpts]
        lz = [p[1] for p in lpts]
        rx = [p[0] for p in rpts]
        rz = [p[1] for p in rpts]

        left_line.set_data(lx, lz)
        right_line.set_data(rx, rz)
        torso_dot.set_data([sway], [stand_h])

        angle_hist["hip"].append(np.degrees(state.left.hip_pitch))
        angle_hist["knee"].append(np.degrees(state.left.knee_pitch))
        angle_hist["ankle"].append(np.degrees(state.left.ankle_pitch))

        for k in angle_hist:
            if len(angle_hist[k]) > max_hist:
                angle_hist[k] = angle_hist[k][-max_hist:]

        n = len(angle_hist["hip"])
        t = np.arange(n) / 30.0
        hip_line.set_data(t, angle_hist["hip"])
        knee_line.set_data(t, angle_hist["knee"])
        ankle_line.set_data(t, angle_hist["ankle"])

        if n > 1:
            ax_angles.set_xlim(max(0, t[-1] - 5), t[-1] + 0.5)

        return [left_line, right_line, torso_dot, hip_line, knee_line, ankle_line]

    anim = FuncAnimation(fig, animate, interval=33, blit=False, cache_frame_data=False)
    plt.show()
