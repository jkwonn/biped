"""cli entry point. run with: python -m biped <command>"""

import argparse
import sys
import json
import numpy as np
from pathlib import Path

from .config import load_config, save_config


def cmd_gait(args):
    """shows an animated stick figure of the gait in a matplotlib window"""
    from .gait import GaitGenerator, GaitCommand
    from .kinematics import forward_kinematics
    from .servo import ServoMapper
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    cfg = load_config(args.config)
    rp = cfg["robot"]
    gp = cfg["gait"]

    gait = GaitGenerator(rp, gp)
    mapper = ServoMapper(cfg.get("servo_calibration"))

    walking = args.walk
    command = GaitCommand(forward_vel=0.03) if walking else GaitCommand()

    print(f"Upper leg:  {rp['upper_leg']*100:.1f} cm")
    print(f"Lower leg:  {rp['lower_leg']*100:.1f} cm")
    print(f"Stand height: {gp['stand_height']*100:.1f} cm")
    print(f"Step period:  {gp['step_period']:.2f} s")
    print(f"Mode: {'Walking' if walking else 'Standing'}")
    print()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    fig.suptitle("Biped Gait — " + ("Walking" if walking else "Standing"))

    ax1.set_aspect("equal")
    ax1.set_title("Side view")
    ax1.set_xlim(-0.12, 0.12)
    ax1.set_ylim(-0.005, 0.20)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color="saddlebrown", lw=2, zorder=1)
    ax1.axhspan(-0.02, 0, color="saddlebrown", alpha=0.15, zorder=0)

    ll, = ax1.plot([], [], "b-o", lw=2, ms=5, label="Left")
    rl, = ax1.plot([], [], "r-o", lw=2, ms=5, label="Right")
    td, = ax1.plot([], [], "ks", ms=10)
    ax1.legend(fontsize=8)

    ax2.set_title("Left leg angles")
    ax2.set_ylim(-90, 130)
    ax2.grid(True, alpha=0.3)
    hist = {"h": [], "k": [], "a": []}

    hl, = ax2.plot([], [], "b-", lw=1.5, label="hip")
    kl, = ax2.plot([], [], "g-", lw=1.5, label="knee")
    al, = ax2.plot([], [], color="orange", lw=1.5, label="ankle")
    ax2.legend(fontsize=8)

    def animate(frame):
        dt = 1 / 30
        state = gait.update(dt, command)

        lpts = forward_kinematics((0, gp["stand_height"]), state.left, rp)
        rpts = forward_kinematics((0, gp["stand_height"]), state.right, rp)

        ll.set_data([p[0] for p in lpts], [p[1] for p in lpts])
        rl.set_data([p[0] for p in rpts], [p[1] for p in rpts])
        td.set_data([0], [gp["stand_height"]])

        hist["h"].append(np.degrees(state.left.hip_pitch))
        hist["k"].append(np.degrees(state.left.knee_pitch))
        hist["a"].append(np.degrees(state.left.ankle_pitch))
        for v in hist.values():
            if len(v) > 150:
                del v[0]

        t = np.arange(len(hist["h"])) / 30
        hl.set_data(t, hist["h"])
        kl.set_data(t, hist["k"])
        al.set_data(t, hist["a"])
        if len(t) > 1:
            ax2.set_xlim(max(0, t[-1] - 5), t[-1] + 0.5)

        # log servo pulses once per second when walking
        if frame % 30 == 0 and walking:
            pulses = mapper.map_legs(state.left, state.right)
            vals = " ".join(f"{v}" for v in pulses.values())
            print(f"phase={state.phase:.2f} PWM: {vals}")

        return [ll, rl, td, hl, kl, al]

    FuncAnimation(fig, animate, interval=33, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


def cmd_tune(args):
    """opens the interactive parameter tuner"""
    from .tuner import run_tuner
    cfg = load_config(args.config)
    run_tuner(cfg)


def cmd_vision(args):
    """runs the person detection and tracking pipeline"""
    from .vision import run_vision
    cfg = load_config(args.config)
    run_vision(
        source=args.source,
        params=cfg["vision"],
        send_udp=args.send_udp,
        udp_ip=args.udp_ip,
        udp_port=args.udp_port,
        export_onnx=args.export_onnx,
    )


def cmd_calibrate(args):
    """prints the current servo calibration values from config"""
    from .servo import ServoMapper
    cfg = load_config(args.config)
    mapper = ServoMapper(cfg.get("servo_calibration"))

    print("Servo Calibration Tool")
    print("=" * 40)
    print()
    print("shows the current calibration values from your config.")
    print("use 'python -m biped run --mode sweep' to test servos on hardware.\n")

    cal = cfg.get("servo_calibration", {})
    for ch_str in sorted(cal.keys(), key=int):
        data = cal[ch_str]
        label = data.get("label", f"servo_{ch_str}")
        center = data.get("center_us", 1500)
        direction = data.get("dir", 1)
        print(f"  CH {ch_str:>2} [{label:>20}]  center={center}us  dir={direction:+d}")

    print()
    print("to adjust, edit configs/default.yaml > servo_calibration")
    print("or save tuned values with: python -m biped export --output my_config.yaml")


def cmd_mock_receiver(args):
    """runs a udp listener that prints incoming commands"""
    from .comms import run_mock_receiver
    run_mock_receiver(args.port)


def cmd_run(args):
    """sends gait commands to the robot hardware over usb serial"""
    import time
    from .gait import GaitGenerator, GaitCommand
    from .servo import ServoMapper
    from .comms import SerialLink

    cfg = load_config(args.config)
    rp = cfg["robot"]
    gp = cfg["gait"]

    gait = GaitGenerator(rp, gp)
    mapper = ServoMapper(cfg.get("servo_calibration"))
    link = SerialLink(args.port)

    mode = args.mode
    hz = rp.get("servo_update_hz", 50)
    dt = 1.0 / hz

    if mode == "stand":
        print("standing, sending pose to servos. ctrl+c to stop.")
        cmd = GaitCommand()
        state = gait.update(dt, cmd)
        pulses = mapper.map_legs(state.left, state.right)
        pulse_list = list(pulses.values())
        link.send_pulses(pulse_list)
        try:
            while True:
                link.poll()
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

    elif mode == "walk":
        yaw = args.yaw or 0.0
        speed = args.speed or 0.03
        print(f"walking, forward={speed:.3f} yaw={yaw:.2f}. ctrl+c to stop.")
        cmd = GaitCommand(forward_vel=speed, yaw_rate=yaw)
        try:
            while True:
                t0 = time.time()
                state = gait.update(dt, cmd)
                pulses = mapper.map_legs(state.left, state.right)
                pulse_list = list(pulses.values())
                link.send_pulses(pulse_list)
                link.poll()

                elapsed = time.time() - t0
                remaining = dt - elapsed
                if remaining > 0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            pass
        # go back to standing when you hit ctrl+c
        print("\nreturning to stand pose...")
        gait.reset()
        state = gait.update(dt, GaitCommand())
        pulses = mapper.map_legs(state.left, state.right)
        link.send_pulses(list(pulses.values()))

    elif mode == "sweep":
        ch = args.channel if args.channel is not None else 0
        print(f"sweeping channel {ch}. ctrl+c to stop.")
        try:
            for pulse in range(1500, 2000, 20):
                link.send_single(ch, pulse)
                print(f"  CH{ch}: {pulse}us")
                time.sleep(0.15)
            for pulse in range(2000, 1000, -20):
                link.send_single(ch, pulse)
                print(f"  CH{ch}: {pulse}us")
                time.sleep(0.15)
            for pulse in range(1000, 1500, 20):
                link.send_single(ch, pulse)
                print(f"  CH{ch}: {pulse}us")
                time.sleep(0.15)
            print(f"  CH{ch}: 1500us (center)")
            link.send_single(ch, 1500)
        except KeyboardInterrupt:
            link.send_single(ch, 1500)

    elif mode == "center":
        print("centering all servos to 1500us.")
        link.center_all()

    else:
        print(f"unknown mode: {mode}")
        return

    link.close()
    print("done.")


def cmd_export(args):
    """saves the current config to a yaml or json file"""
    cfg = load_config(args.config)
    save_config(cfg, args.output)


def main():
    parser = argparse.ArgumentParser(
        prog="biped",
        description="bipedal desk robot cli toolkit",
    )
    parser.add_argument("--config", default=None, help="path to config yaml/json (merges with defaults)")
    sub = parser.add_subparsers(dest="command", help="available commands")

    p_run = sub.add_parser("run", help="send gait commands to hardware over serial")
    p_run.add_argument("--port", required=True, help="serial port (e.g. /dev/cu.usbmodem14101)")
    p_run.add_argument("--mode", default="stand", choices=["stand", "walk", "sweep", "center"],
                        help="operation mode")
    p_run.add_argument("--yaw", type=float, default=0.0, help="yaw rate for walking (-1 to 1)")
    p_run.add_argument("--speed", type=float, default=0.03, help="forward velocity (m/s)")
    p_run.add_argument("--channel", type=int, default=None, help="servo channel for sweep mode")

    p_gait = sub.add_parser("gait", help="run gait visualization")
    p_gait.add_argument("--walk", action="store_true", help="start in walking mode")
    p_gait.add_argument("--stand", action="store_true", help="start in standing mode (default)")

    p_tune = sub.add_parser("tune", help="interactive parameter tuner with live visualization")

    p_vis = sub.add_parser("vision", help="run person detection and tracking")
    p_vis.add_argument("--source", default="0", help="video source: webcam index, file, or mjpeg url")
    p_vis.add_argument("--send-udp", action="store_true", help="send commands over udp")
    p_vis.add_argument("--udp-ip", default="127.0.0.1", help="udp target ip")
    p_vis.add_argument("--udp-port", type=int, default=8888, help="udp target port")
    p_vis.add_argument("--export-onnx", action="store_true", help="export yolo to onnx and exit")

    p_cal = sub.add_parser("calibrate", help="show servo calibration values")

    p_mock = sub.add_parser("mock-receiver", help="run mock udp command receiver")
    p_mock.add_argument("--port", type=int, default=8888, help="udp port to listen on")

    p_exp = sub.add_parser("export", help="export config to yaml")
    p_exp.add_argument("--output", default="configs/tuned.yaml", help="output file path")

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        return

    handlers = {
        "run": cmd_run,
        "gait": cmd_gait,
        "tune": cmd_tune,
        "vision": cmd_vision,
        "calibrate": cmd_calibrate,
        "mock-receiver": cmd_mock_receiver,
        "export": cmd_export,
    }

    handlers[args.command](args)


if __name__ == "__main__":
    main()
