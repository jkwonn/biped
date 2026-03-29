"""
standalone servo test. sweeps servos through their range.

this is for use on the actual robot hardware with a pca9685.
on your dev machine it just prints the pulse values.

usage:
    python scripts/servo_test.py
    python scripts/servo_test.py --channel 3 --min 600 --max 2400
"""

import argparse
import time
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from biped.servo import ServoMapper, DS3240_PULSE_MIN, DS3240_PULSE_MAX
from biped.config import load_config


def sweep_test(channel: int, pulse_min: int, pulse_max: int, steps: int = 20, delay: float = 0.1):
    """sweeps a servo channel from min to max and back"""
    print(f"sweeping channel {channel}: {pulse_min}us > {pulse_max}us > {pulse_min}us")
    print(f"  {steps} steps, {delay}s per step")
    print()

    for pulse in range(pulse_min, pulse_max, (pulse_max - pulse_min) // steps):
        print(f"  CH{channel}: {pulse}us")
        time.sleep(delay)

    for pulse in range(pulse_max, pulse_min, -(pulse_max - pulse_min) // steps):
        print(f"  CH{channel}: {pulse}us")
        time.sleep(delay)

    center = (pulse_min + pulse_max) // 2
    print(f"  CH{channel}: {center}us (center)")
    print("done.")


def center_all(cfg: dict):
    """sends all servos to their calibrated center positions"""
    mapper = ServoMapper(cfg.get("servo_calibration"))
    centers = mapper.get_all_pulses_at_zero()

    print("centering all servos:")
    for label, pulse in centers.items():
        print(f"  {label:>20}: {pulse}us")
    print()
    print("on real hardware this would command all servos to their neutral position.")
    print("use this to verify your servo horn mounting.")


def main():
    parser = argparse.ArgumentParser(description="servo test utility")
    parser.add_argument("--channel", type=int, default=-1, help="servo channel to test (-1 = all centers)")
    parser.add_argument("--min", type=int, default=DS3240_PULSE_MIN, help="min pulse width (us)")
    parser.add_argument("--max", type=int, default=DS3240_PULSE_MAX, help="max pulse width (us)")
    parser.add_argument("--config", default=None, help="config file path")

    args = parser.parse_args()
    cfg = load_config(args.config)

    if args.channel == -1:
        center_all(cfg)
    else:
        sweep_test(args.channel, args.min, args.max)


if __name__ == "__main__":
    main()
