"""imu based pid balance correction layer"""

import numpy as np


class PIDController:
    """
    single axis pid controller with integral windup clamping
    and output rate limiting.

    tuning guide:
        1. set ki and kd to 0, increase kp until the robot corrects
           but starts to oscillate
        2. back kp off ~30%, then increase kd until oscillation damps out
        3. add a small ki if there's a steady state lean the robot
           can't fully correct
    """

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, integral_max=0.5, output_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.output_max = output_max

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_output = 0.0

    def update(self, error: float, dt: float) -> float:
        """compute the pid output for one timestep"""
        # proportional
        p = self.kp * error

        # integral with anti-windup clamp
        self._integral += error * dt
        self._integral = np.clip(self._integral, -self.integral_max, self.integral_max)
        i = self.ki * self._integral

        # derivative (on error, filtered by dt to avoid spikes)
        d = 0.0
        if dt > 0:
            d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        output = np.clip(p + i + d, -self.output_max, self.output_max)
        self._prev_output = output
        return output

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_output = 0.0


class BalanceController:
    """
    reads the imu's pitch and roll, runs a pid loop for each axis,
    and outputs joint angle corrections to layer on top of the gait.

    pitch pid adjusts hip_pitch (forward/backward lean).
    roll pid adjusts hip_roll (lateral tilt).

    start with very small gains and increase until you see correction
    without oscillation on the real robot. see PIDController for
    tuning steps.
    """

    def __init__(
        self,
        pitch_kp=0.3, pitch_ki=0.05, pitch_kd=0.1,
        roll_kp=0.2, roll_ki=0.03, roll_kd=0.08,
        output_max=0.3,
    ):
        self.pitch_pid = PIDController(
            kp=pitch_kp, ki=pitch_ki, kd=pitch_kd, output_max=output_max,
        )
        self.roll_pid = PIDController(
            kp=roll_kp, ki=roll_ki, kd=roll_kd, output_max=output_max,
        )
        self.pitch_target = 0.0
        self.roll_target = 0.0

    def compute(
        self,
        imu_pitch: float,
        imu_roll: float,
        expected_roll: float,
        dt: float,
    ) -> dict:
        """
        takes the measured imu angles, compares against what we expect,
        and returns correction offsets to add to the gait joint angles.

        expected_roll accounts for the intentional lateral sway during
        walking so we don't fight the gait generator.
        """
        pitch_err = self.pitch_target - imu_pitch
        roll_err = (self.roll_target + expected_roll) - imu_roll

        pitch_corr = self.pitch_pid.update(pitch_err, dt)
        roll_corr = self.roll_pid.update(roll_err, dt)

        return {
            "hip_pitch_offset": pitch_corr,
            "left_hip_roll_offset": -roll_corr,
            "right_hip_roll_offset": roll_corr,
        }

    def reset(self):
        self.pitch_pid.reset()
        self.roll_pid.reset()
