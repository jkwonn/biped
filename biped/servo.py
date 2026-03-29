"""maps joint angles to servo pwm pulse widths"""
import numpy as np
from typing import Optional
from .kinematics import LegIKResult


# ds3240 servo specs
DS3240_PULSE_MIN = 500    # microseconds at 0 degrees
DS3240_PULSE_MAX = 2500   # microseconds at 270 degrees
DS3240_ANGLE_RANGE = 4.712  # 270 degrees in radians


class ServoMapper:
    """
    converts joint angles (radians) to pwm pulse widths (microseconds)
    for each servo channel on the pca9685.

    each servo has its own calibration:
        center_us: pulse width at the joint's zero position
        us_per_rad: how many microseconds per radian of rotation
        dir: +1 or -1 to flip the rotation direction
    """

    def __init__(self, calibration: Optional[dict] = None):
        self.default_center = 1500
        self.default_us_per_rad = (DS3240_PULSE_MAX - DS3240_PULSE_MIN) / DS3240_ANGLE_RANGE

        self.cal = {}
        if calibration:
            for ch_str, data in calibration.items():
                ch = int(ch_str)
                self.cal[ch] = {
                    "center_us": data.get("center_us", self.default_center),
                    "us_per_rad": data.get("us_per_rad", self.default_us_per_rad),
                    "dir": data.get("dir", 1),
                    "label": data.get("label", f"servo_{ch}"),
                }

    def angle_to_pulse(self, angle_rad: float, channel: int = -1) -> int:
        """converts a joint angle in radians to a pwm pulse width in microseconds"""
        if channel in self.cal:
            c = self.cal[channel]
            pulse = c["center_us"] + c["dir"] * angle_rad * c.get("us_per_rad", self.default_us_per_rad)
        else:
            pulse = self.default_center + angle_rad * self.default_us_per_rad

        return int(np.clip(pulse, DS3240_PULSE_MIN, DS3240_PULSE_MAX))

    def pulse_to_angle(self, pulse_us: int, channel: int = -1) -> float:
        """converts a pwm pulse width back to a joint angle in radians"""
        if channel in self.cal:
            c = self.cal[channel]
            return c["dir"] * (pulse_us - c["center_us"]) / c.get("us_per_rad", self.default_us_per_rad)
        else:
            return (pulse_us - self.default_center) / self.default_us_per_rad

    def map_legs(self, left: LegIKResult, right: LegIKResult) -> dict:
        """converts both legs' joint angles to pulse widths using the channel mapping"""
        joints = [
            (0, left.hip_roll),
            (1, left.hip_pitch),
            (2, left.knee_pitch),
            (3, left.ankle_pitch),
            (4, right.hip_roll),
            (5, right.hip_pitch),
            (6, right.knee_pitch),
            (7, right.ankle_pitch),
        ]
        result = {}
        for ch, angle in joints:
            label = self.cal.get(ch, {}).get("label", f"servo_{ch}")
            result[label] = self.angle_to_pulse(angle, ch)
        return result

    def set_calibration(self, channel: int, center_us: int, direction: int = 1):
        """sets or updates calibration for a single servo channel"""
        if channel not in self.cal:
            self.cal[channel] = {
                "center_us": center_us,
                "us_per_rad": self.default_us_per_rad,
                "dir": direction,
                "label": f"servo_{channel}",
            }
        else:
            self.cal[channel]["center_us"] = center_us
            self.cal[channel]["dir"] = direction

    def get_all_pulses_at_zero(self) -> dict:
        """returns the pulse width each servo sends when all joints are at zero"""
        result = {}
        for ch, data in self.cal.items():
            result[data["label"]] = data["center_us"]
        return result
