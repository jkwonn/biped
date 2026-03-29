"""parametric quasi-static walking gait generator.

turning uses differential stepping where one leg takes shorter steps
than the other, making the robot walk in an arc.

    yaw_rate > 0  means turn left  (right leg steps longer)
    yaw_rate < 0  means turn right (left leg steps longer)
    yaw_rate = 1  means the inner leg stays still (pivot turn)
"""

import numpy as np
import warnings
from dataclasses import dataclass
from typing import Optional, Tuple

from .kinematics import leg_ik, LegIKResult


@dataclass
class GaitCommand:
    """high level velocity command for the robot"""
    forward_vel: float = 0.0
    yaw_rate: float = 0.0


@dataclass
class GaitState:
    """full gait output for one timestep"""
    left: LegIKResult = None
    right: LegIKResult = None
    phase: float = 0.0
    torso_roll: float = 0.0
    left_foot: Tuple[float, float, float] = (0, 0, 0)
    right_foot: Tuple[float, float, float] = (0, 0, 0)
    is_walking: bool = False


class GaitGenerator:
    """
    produces joint angle trajectories from velocity commands.

    all physical dimensions come from robot_params and gait_params dicts.
    plug in your measurements and tune the gait params for your robot.

    turning works by scaling step length per leg:
        left_step  = step_length * (1 - yaw_rate)
        right_step = step_length * (1 + yaw_rate)
    """

    def __init__(self, robot_params: dict, gait_params: dict):
        self.rp = robot_params
        self.gp = gait_params
        self.phase = 0.0
        self.path_x = 0.0

    def reset(self):
        self.phase = 0.0
        self.path_x = 0.0

    def update_params(self, robot_params: dict = None, gait_params: dict = None):
        """swap in new parameters without resetting the gait phase"""
        if robot_params is not None:
            self.rp.update(robot_params)
        if gait_params is not None:
            self.gp.update(gait_params)

    def _foot_trajectory(
        self, leg_phase: float, stance_x: float, stance_y: float, step_len: float,
    ) -> Tuple[float, float, float]:
        duty = np.clip(self.gp["duty_factor"], 0.01, 0.99)
        step_h = self.gp["step_height"]
        stand_h = self.gp["stand_height"] - self.rp["foot_height"]

        if leg_phase < duty:
            # stance phase: foot on the ground, sliding backward
            progress = leg_phase / duty
            foot_x = stance_x + step_len * (0.5 - progress)
            foot_z = -stand_h
        else:
            # swing phase: foot in the air, moving forward
            progress = (leg_phase - duty) / (1.0 - duty)
            foot_x = stance_x + step_len * (-0.5 + progress)
            foot_z = -stand_h + step_h * np.sin(np.pi * progress)

        return foot_x, stance_y, foot_z

    def _lateral_sway(self, phase: float) -> float:
        sway_amp = self.gp["lateral_sway"]
        advance = self.gp["sway_advance"]
        return sway_amp * np.sin(2 * np.pi * (phase + advance))

    def update(self, dt: float, command: Optional[GaitCommand] = None) -> GaitState:
        if command is None:
            command = GaitCommand()

        period = max(self.gp["step_period"], 0.01)
        speed = abs(command.forward_vel)
        is_walking = speed > 0.01

        state = GaitState()
        state.is_walking = is_walking

        if is_walking:
            step_len = self.gp["step_length"]
            max_speed = step_len / period
            speed_scale = np.clip(speed / max_speed, 0.0, 1.0)

            # scale step length per leg for turning
            yaw = np.clip(command.yaw_rate, -1.0, 1.0)
            left_step = step_len * (1.0 - yaw)
            right_step = step_len * (1.0 + yaw)

            self.phase += dt / period
            self.phase %= 1.0
            self.path_x += command.forward_vel * dt

            left_phase = self.phase % 1.0
            right_phase = (self.phase + 0.5) % 1.0
            sway = self._lateral_sway(self.phase)
            spread = self.gp["stand_foot_spread"]

            lean = self.gp.get("forward_lean", 0.0) * speed_scale
            lx, ly, lz = self._foot_trajectory(left_phase, lean, spread, left_step)
            rx, ry, rz = self._foot_trajectory(right_phase, lean, -spread, right_step)

            ly -= sway
            ry -= sway
        else:
            spread = self.gp["stand_foot_spread"]
            stand_h = self.gp["stand_height"] - self.rp["foot_height"]

            lx, ly, lz = 0.0, spread, -stand_h
            rx, ry, rz = 0.0, -spread, -stand_h
            sway = 0.0

        # foot positions here are relative to body center, so we offset
        # by the hip's lateral position to get positions relative to each hip
        state.left = leg_ik(
            lx, ly - self.rp["hip_offset_y"], lz, self.rp,
        )
        state.right = leg_ik(
            rx, ry + self.rp["hip_offset_y"], rz, self.rp,
        )

        if not state.left.reachable or not state.right.reachable:
            unreachable = []
            if not state.left.reachable:
                unreachable.append("left")
            if not state.right.reachable:
                unreachable.append("right")
            warnings.warn(
                f"ik target unreachable for {', '.join(unreachable)} leg(s), "
                f"angles clamped so servo positions may be wrong",
                stacklevel=2,
            )

        state.phase = self.phase
        state.torso_roll = np.arctan2(sway, self.gp["stand_height"]) if is_walking else 0.0
        state.left_foot = (lx, ly, lz)
        state.right_foot = (rx, ry, rz)

        return state
