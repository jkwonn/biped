"""tests for ik and fk math"""
import numpy as np
import pytest
from biped.kinematics import leg_ik, forward_kinematics, LegIKResult
from biped.config import load_config


@pytest.fixture
def params():
    """default robot params from config"""
    return load_config()["robot"]


@pytest.fixture
def gait_params():
    return load_config()["gait"]


class TestLegIK:

    def test_standing_directly_below_hip(self, params):
        """foot directly below hip should give zero roll and be reachable"""
        stand_h = 0.13 - params["foot_height"]
        result = leg_ik(0.0, 0.0, -stand_h, params)

        assert result.reachable
        assert abs(result.hip_roll) < 1e-6
        # ankle should cancel hip + knee to keep the foot flat
        assert abs(result.ankle_pitch + result.hip_pitch + result.knee_pitch) < 0.01

    def test_standing_with_lateral_offset(self, params):
        """foot offset to the side should produce a nonzero roll angle"""
        result = leg_ik(0.0, 0.04, -0.115, params)

        assert result.reachable
        assert result.hip_roll != 0.0
        expected_roll = np.arctan2(-0.04, 0.115)
        assert abs(result.hip_roll - expected_roll) < 1e-6

    def test_foot_forward(self, params):
        """foot pushed forward should change hip_pitch but not roll"""
        result = leg_ik(0.02, 0.0, -0.115, params)

        assert result.reachable
        assert abs(result.hip_roll) < 1e-6
        assert result.hip_pitch != 0.0

    def test_unreachable_target(self, params):
        """foot beyond max reach should be flagged but still return finite angles"""
        max_reach = params["upper_leg"] + params["lower_leg"]
        result = leg_ik(0.0, 0.0, -(max_reach + 0.1), params)

        assert not result.reachable
        assert np.all(np.isfinite(result.as_array()))

    def test_degenerate_at_hip(self, params):
        """foot right at the hip origin should not crash"""
        result = leg_ik(0.0, 0.0, 0.0, params)

        # with equal leg lengths min_reach is 0, so this is technically
        # reachable (fully folded). either way angles must be finite.
        assert np.all(np.isfinite(result.as_array()))

    def test_ankle_keeps_foot_flat(self, params):
        """ankle should cancel out hip + knee so the sole stays level"""
        for fx, fy, fz in [(0.0, 0.0, -0.10), (0.02, 0.03, -0.11), (-0.01, -0.02, -0.12)]:
            result = leg_ik(fx, fy, fz, params)
            if result.reachable:
                total = result.hip_pitch + result.knee_pitch + result.ankle_pitch
                assert abs(total) < 0.1, f"foot not flat: total={total:.3f} for ({fx},{fy},{fz})"

    def test_joint_limits_respected(self, params):
        """all output angles should stay within the configured limits"""
        for fx, fy, fz in [(0.0, 0.05, -0.08), (0.03, 0.0, -0.14), (0.0, 0.0, -0.04)]:
            result = leg_ik(fx, fy, fz, params)
            assert params["hip_roll_min"] <= result.hip_roll <= params["hip_roll_max"]
            assert params["hip_pitch_min"] <= result.hip_pitch <= params["hip_pitch_max"]
            assert params["knee_pitch_min"] <= result.knee_pitch <= params["knee_pitch_max"]
            assert params["ankle_pitch_min"] <= result.ankle_pitch <= params["ankle_pitch_max"]


class TestForwardKinematics:

    def test_standing_foot_at_ground(self, params, gait_params):
        """foot tip should land at roughly z=0 when standing normally"""
        stand_h = gait_params["stand_height"] - params["foot_height"]
        ik = leg_ik(0.0, 0.0, -stand_h, params)
        pts = forward_kinematics((0, gait_params["stand_height"]), ik, params)

        hip, knee, ankle, foot = pts
        assert abs(foot[1]) < 0.005, f"foot z={foot[1]:.4f}, expected ~0"

    def test_chain_order(self, params):
        """joints should go top to bottom: hip > knee > ankle > foot"""
        ik = leg_ik(0.0, 0.0, -0.10, params)
        pts = forward_kinematics((0, 0.13), ik, params)

        for i in range(len(pts) - 1):
            assert pts[i][1] > pts[i + 1][1], (
                f"joint {i} z={pts[i][1]:.4f} not above joint {i+1} z={pts[i+1][1]:.4f}"
            )

    def test_fk_with_roll(self, params):
        """roll should compress the vertical extent since we're projecting to 2d"""
        ik_no_roll = leg_ik(0.0, 0.0, -0.115, params)
        ik_with_roll = leg_ik(0.0, 0.04, -0.115, params)

        pts_no_roll = forward_kinematics((0, 0.13), ik_no_roll, params)
        pts_with_roll = forward_kinematics((0, 0.13), ik_with_roll, params)

        # with roll the leg looks shorter in the side view, so the foot
        # ends up slightly higher than without roll
        assert pts_with_roll[3][1] > pts_no_roll[3][1]
