"""imu based reactive balance correction layer"""
import numpy as np


class BalanceController:
    """
    reads the imu's pitch and roll, then computes small corrections
    to add on top of the gait generator's joint angles.

    pitch correction adjusts hip_pitch (forward/backward lean).
    roll correction adjusts hip_roll (lateral tilt).

    start with very small gains and increase until you see correction
    without oscillation on the real robot.
    """

    def __init__(self, pitch_gain=0.3, roll_gain=0.2, max_rate=1.0):
        self.pitch_gain = pitch_gain
        self.roll_gain = roll_gain
        self.max_rate = max_rate

        self.pitch_target = 0.0
        self.roll_target = 0.0

        self._prev_pitch = 0.0
        self._prev_roll = 0.0

    def compute(
        self,
        imu_pitch: float,
        imu_roll: float,
        expected_roll: float,
        dt: float,
    ) -> dict:
        """
        figures out how much to nudge each joint to keep the robot upright.

        takes the measured imu angles, compares against what we expect,
        and returns correction offsets to add to the gait joint angles.
        rate limited so it doesn't jerk around.
        """
        pitch_err = self.pitch_target - imu_pitch
        roll_err = (self.roll_target + expected_roll) - imu_roll

        pitch_corr = self.pitch_gain * pitch_err
        roll_corr = self.roll_gain * roll_err

        # rate limit so corrections don't jump too fast between frames
        max_d = self.max_rate * dt
        pitch_corr = self._prev_pitch + np.clip(
            pitch_corr - self._prev_pitch, -max_d, max_d
        )
        roll_corr = self._prev_roll + np.clip(
            roll_corr - self._prev_roll, -max_d, max_d
        )

        self._prev_pitch = pitch_corr
        self._prev_roll = roll_corr

        return {
            "hip_pitch_offset": pitch_corr,
            "left_hip_roll_offset": -roll_corr,
            "right_hip_roll_offset": roll_corr,
        }

    def reset(self):
        self._prev_pitch = 0.0
        self._prev_roll = 0.0
