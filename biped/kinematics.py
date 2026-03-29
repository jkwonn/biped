"""analytical inverse and forward kinematics for a 4-DOF bipedal leg.

joint chain: hip_roll (x axis) > hip_pitch (y axis) > knee_pitch (y axis) > ankle_pitch (y axis)

coordinate frame (relative to hip joint):
    x = forward  (positive)
    y = lateral   (positive = outward from torso)
    z = vertical  (negative = downward)
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class LegIKResult:
    """joint angles for one leg (radians)"""
    hip_roll: float = 0.0
    hip_pitch: float = 0.0
    knee_pitch: float = 0.0
    ankle_pitch: float = 0.0
    reachable: bool = True

    def as_array(self) -> np.ndarray:
        return np.array([self.hip_roll, self.hip_pitch, self.knee_pitch, self.ankle_pitch])

    def as_degrees(self) -> dict:
        return {
            "hip_roll": np.degrees(self.hip_roll),
            "hip_pitch": np.degrees(self.hip_pitch),
            "knee_pitch": np.degrees(self.knee_pitch),
            "ankle_pitch": np.degrees(self.ankle_pitch),
        }


def leg_ik(
    foot_x: float,
    foot_y: float,
    foot_z: float,
    params: dict,
) -> LegIKResult:
    """
    solves the 4 joint angles needed to place the foot at (x, y, z)
    relative to the hip.

    hip_roll swings the leg laterally in the frontal (y/z) plane.
    then hip_pitch, knee, and ankle work in the sagittal plane of
    the rolled leg like a normal 2-link arm.
    """
    result = LegIKResult()
    L1 = params["upper_leg"]
    L2 = params["lower_leg"]

    # figure out the roll angle that points the leg toward the foot's
    # lateral position. foot_z is negative (down), so we negate both
    # to get a clean atan2 in the right quadrant.
    result.hip_roll = np.arctan2(-foot_y, -foot_z)

    # now project into the rolled leg plane. the distance from hip to
    # foot in the y/z plane becomes our "downward" reach target.
    foot_down = np.sqrt(foot_y ** 2 + foot_z ** 2)
    foot_forward = foot_x

    # standard 2-link ik for hip_pitch and knee_pitch
    dist_sq = foot_forward ** 2 + foot_down ** 2
    dist = np.sqrt(dist_sq)

    max_reach = (L1 + L2) * 0.999
    min_reach = abs(L1 - L2) * 1.001
    if dist > max_reach or dist < min_reach:
        result.reachable = False
        dist = np.clip(dist, min_reach * 1.01, max_reach * 0.99)
        dist_sq = dist ** 2

    cos_knee = (dist_sq - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)
    knee_angle = np.arccos(cos_knee)

    alpha = np.arctan2(foot_forward, foot_down)
    beta = np.arctan2(L2 * np.sin(knee_angle), L1 + L2 * np.cos(knee_angle))

    result.hip_pitch = alpha - beta
    result.knee_pitch = np.pi - knee_angle

    # ankle cancels out hip + knee so the foot stays flat on the ground
    result.ankle_pitch = -(result.hip_pitch + result.knee_pitch)

    # clamp everything to the physical joint limits
    result.hip_roll = np.clip(result.hip_roll, params["hip_roll_min"], params["hip_roll_max"])
    result.hip_pitch = np.clip(result.hip_pitch, params["hip_pitch_min"], params["hip_pitch_max"])
    result.knee_pitch = np.clip(result.knee_pitch, params["knee_pitch_min"], params["knee_pitch_max"])
    result.ankle_pitch = np.clip(result.ankle_pitch, params["ankle_pitch_min"], params["ankle_pitch_max"])

    return result


def forward_kinematics(hip_pos: tuple, ik: LegIKResult, params: dict) -> list:
    """
    takes the hip position and joint angles, returns the (x, z) positions
    of each joint for the side view visualization.

    returns [hip, knee, ankle, foot_tip]. the roll compresses the vertical
    extent by cos(roll) since we're projecting the tilted leg onto the
    sagittal plane.
    """
    L1 = params["upper_leg"]
    L2 = params["lower_leg"]
    fh = params["foot_height"]
    hx, hz = hip_pos

    cos_roll = np.cos(ik.hip_roll)

    # upper leg projected to the side view
    kx = hx + L1 * np.sin(ik.hip_pitch)
    kz = hz - L1 * np.cos(ik.hip_pitch) * cos_roll

    # lower leg
    lower_abs = ik.hip_pitch + ik.knee_pitch
    ax = kx + L2 * np.sin(lower_abs)
    az = kz - L2 * np.cos(lower_abs) * cos_roll

    # foot tip hangs straight down from the ankle
    return [(hx, hz), (kx, kz), (ax, az), (ax, az - fh)]


def compute_workspace(params: dict, resolution: int = 50) -> np.ndarray:
    """
    maps the reachable workspace of one leg in the sagittal plane.
    returns an array of (x, z, reachable) points. assumes the foot
    is directly below the hip (foot_y = 0).
    """
    L1 = params["upper_leg"]
    L2 = params["lower_leg"]
    max_r = L1 + L2
    points = []

    for x in np.linspace(-max_r, max_r, resolution):
        for z in np.linspace(-max_r * 1.2, 0, resolution):
            result = leg_ik(x, 0.0, z, params)
            points.append((x, z, result.reachable))

    return np.array(points)
