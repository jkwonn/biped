"""tests for the gait generator and differential stepping"""
import numpy as np
import pytest
import warnings
from biped.gait import GaitGenerator, GaitCommand
from biped.config import load_config


@pytest.fixture
def gait():
    cfg = load_config()
    return GaitGenerator(cfg["robot"], cfg["gait"])


class TestGaitGenerator:

    def test_standing_produces_valid_ik(self, gait):
        """standing with no command should be reachable for both legs"""
        state = gait.update(1 / 30)
        assert state.left.reachable
        assert state.right.reachable
        assert not state.is_walking

    def test_walking_forward(self, gait):
        """walking should advance the phase and stay reachable"""
        cmd = GaitCommand(forward_vel=0.03)
        states = [gait.update(1 / 30, cmd) for _ in range(30)]

        assert states[-1].is_walking
        assert states[-1].phase > 0
        assert all(s.left.reachable for s in states)
        assert all(s.right.reachable for s in states)

    def test_differential_stepping_straight(self, gait):
        """no yaw should give both legs the same step range"""
        cmd = GaitCommand(forward_vel=0.03, yaw_rate=0.0)

        left_xs, right_xs = [], []
        for _ in range(60):
            state = gait.update(1 / 30, cmd)
            left_xs.append(state.left_foot[0])
            right_xs.append(state.right_foot[0])

        left_range = max(left_xs) - min(left_xs)
        right_range = max(right_xs) - min(right_xs)
        assert abs(left_range - right_range) < 0.001

    def test_differential_stepping_turn_left(self, gait):
        """positive yaw should make the right leg step further than the left"""
        cmd = GaitCommand(forward_vel=0.03, yaw_rate=0.5)

        left_xs, right_xs = [], []
        for _ in range(60):
            state = gait.update(1 / 30, cmd)
            left_xs.append(state.left_foot[0])
            right_xs.append(state.right_foot[0])

        left_range = max(left_xs) - min(left_xs)
        right_range = max(right_xs) - min(right_xs)
        assert right_range > left_range

    def test_differential_stepping_turn_right(self, gait):
        """negative yaw should make the left leg step further than the right"""
        cmd = GaitCommand(forward_vel=0.03, yaw_rate=-0.5)

        left_xs, right_xs = [], []
        for _ in range(60):
            state = gait.update(1 / 30, cmd)
            left_xs.append(state.left_foot[0])
            right_xs.append(state.right_foot[0])

        left_range = max(left_xs) - min(left_xs)
        right_range = max(right_xs) - min(right_xs)
        assert left_range > right_range

    def test_yaw_rate_clamped(self, gait):
        """extreme yaw values should get clamped and not crash"""
        cmd = GaitCommand(forward_vel=0.03, yaw_rate=5.0)
        state = gait.update(1 / 30, cmd)
        assert state.left.reachable
        assert state.right.reachable

    def test_duty_factor_extremes(self, gait):
        """duty at 0 or 1 should not crash because we clamp it internally"""
        gait.gp["duty_factor"] = 0.0
        cmd = GaitCommand(forward_vel=0.03)
        state = gait.update(1 / 30, cmd)
        assert np.all(np.isfinite(state.left.as_array()))

        gait.gp["duty_factor"] = 1.0
        state = gait.update(1 / 30, cmd)
        assert np.all(np.isfinite(state.left.as_array()))

    def test_step_period_zero(self, gait):
        """zero period should not crash because we floor it to 0.01"""
        gait.gp["step_period"] = 0.0
        cmd = GaitCommand(forward_vel=0.03)
        state = gait.update(1 / 30, cmd)
        assert np.all(np.isfinite(state.left.as_array()))

    def test_unreachable_warns(self, gait):
        """should warn when the foot target is beyond what the legs can reach"""
        gait.gp["stand_height"] = 0.5
        cmd = GaitCommand(forward_vel=0.03)
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            gait.update(1 / 30, cmd)
            assert len(w) >= 1
            assert "unreachable" in str(w[0].message).lower()
