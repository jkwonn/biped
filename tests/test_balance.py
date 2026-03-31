"""tests for the pid balance controller"""

import numpy as np
import pytest
from biped.balance import PIDController, BalanceController


class TestPIDController:

    def test_proportional_only(self):
        """p term should push output proportional to the error"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        out = pid.update(0.5, dt=0.02)
        assert abs(out - 0.5) < 1e-6

    def test_integral_accumulates(self):
        """repeated error should build up the integral term"""
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
        for _ in range(10):
            out = pid.update(1.0, dt=0.1)
        # after 10 steps of error=1.0, dt=0.1: integral = 1.0 (clamped to 0.5)
        assert out >= 0.5

    def test_integral_windup_clamp(self):
        """integral should not grow beyond integral_max"""
        pid = PIDController(kp=0.0, ki=10.0, kd=0.0, integral_max=0.1)
        for _ in range(100):
            pid.update(1.0, dt=0.1)
        # even with huge ki and many steps, output is bounded
        assert abs(pid._integral) <= 0.1 + 1e-6

    def test_derivative_responds_to_change(self):
        """d term should respond to changes in error"""
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0)
        pid.update(0.0, dt=0.02)
        out = pid.update(0.1, dt=0.02)
        # error changed by 0.1 over 0.02s, so d = 1.0 * 0.1/0.02 = 5.0
        # but clamped to output_max=1.0
        assert out > 0

    def test_output_clamped(self):
        """output should never exceed output_max"""
        pid = PIDController(kp=100.0, output_max=0.5)
        out = pid.update(1.0, dt=0.02)
        assert abs(out) <= 0.5 + 1e-6

    def test_reset_clears_state(self):
        """reset should zero out integral and previous error"""
        pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
        pid.update(1.0, dt=0.02)
        pid.update(0.5, dt=0.02)
        pid.reset()
        assert pid._integral == 0.0
        assert pid._prev_error == 0.0

    def test_zero_error_gives_zero_output(self):
        """no error should produce no correction"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        out = pid.update(0.0, dt=0.02)
        assert out == 0.0


class TestBalanceController:

    def test_no_tilt_no_correction(self):
        """robot perfectly upright should get zero corrections"""
        bal = BalanceController()
        result = bal.compute(imu_pitch=0.0, imu_roll=0.0, expected_roll=0.0, dt=0.02)
        assert abs(result["hip_pitch_offset"]) < 1e-6
        assert abs(result["left_hip_roll_offset"]) < 1e-6
        assert abs(result["right_hip_roll_offset"]) < 1e-6

    def test_forward_lean_corrects_pitch(self):
        """leaning forward should produce a negative pitch correction"""
        bal = BalanceController(pitch_kp=1.0, pitch_ki=0.0, pitch_kd=0.0)
        result = bal.compute(imu_pitch=0.1, imu_roll=0.0, expected_roll=0.0, dt=0.02)
        assert result["hip_pitch_offset"] < 0

    def test_roll_correction_is_opposite_per_leg(self):
        """left and right hip roll offsets should be opposite signs"""
        bal = BalanceController(roll_kp=1.0, roll_ki=0.0, roll_kd=0.0)
        result = bal.compute(imu_pitch=0.0, imu_roll=0.05, expected_roll=0.0, dt=0.02)
        assert result["left_hip_roll_offset"] == -result["right_hip_roll_offset"]

    def test_expected_roll_offsets_target(self):
        """during sway the expected roll should not trigger corrections"""
        bal = BalanceController(roll_kp=1.0, roll_ki=0.0, roll_kd=0.0)
        # imu reads 0.05 roll, but gait expects 0.05 roll, so error = 0
        result = bal.compute(imu_pitch=0.0, imu_roll=0.05, expected_roll=0.05, dt=0.02)
        assert abs(result["left_hip_roll_offset"]) < 1e-6

    def test_reset_clears_both_pids(self):
        """reset should clear state in both pitch and roll pid loops"""
        bal = BalanceController()
        bal.compute(imu_pitch=0.1, imu_roll=0.1, expected_roll=0.0, dt=0.02)
        bal.reset()
        assert bal.pitch_pid._integral == 0.0
        assert bal.roll_pid._integral == 0.0
