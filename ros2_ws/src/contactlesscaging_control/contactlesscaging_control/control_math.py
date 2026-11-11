"""Pure formation-control mathematics, free of ROS dependencies."""
from __future__ import annotations
import math
from dataclasses import dataclass


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def heading_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw for a ROS quaternion (x,y,z,w)."""
    return math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0*(y*y + z*z))


@dataclass
class PID2D:
    kp: float
    ki: float
    kd: float
    integral_limit: float = 0.25
    derivative_filter: float = 0.25

    def __post_init__(self):
        self.ix = 0.0
        self.iy = 0.0
        self.ex_prev = None
        self.ey_prev = None
        self.dx_prev = 0.0
        self.dy_prev = 0.0

    def reset(self):
        self.ix = self.iy = 0.0
        self.ex_prev = self.ey_prev = None
        self.dx_prev = self.dy_prev = 0.0

    def update(self, ex: float, ey: float, dt: float) -> tuple[float, float]:
        if dt <= 0.0 or dt > 1.0:
            self.reset()
            return 0.0, 0.0
        self.ix = clamp(self.ix + ex * dt, -self.integral_limit, self.integral_limit)
        self.iy = clamp(self.iy + ey * dt, -self.integral_limit, self.integral_limit)
        raw_dx = (ex - self.ex_prev) / dt if self.ex_prev is not None else 0.0
        raw_dy = (ey - self.ey_prev) / dt if self.ey_prev is not None else 0.0
        a = clamp(self.derivative_filter, 0.0, 1.0)
        self.dx_prev = a * raw_dx + (1-a) * self.dx_prev
        self.dy_prev = a * raw_dy + (1-a) * self.dy_prev
        self.ex_prev, self.ey_prev = ex, ey
        return (self.kp*ex + self.ki*self.ix + self.kd*self.dx_prev,
                self.kp*ey + self.ki*self.iy + self.kd*self.dy_prev)


def planar_to_cmd(qx: float, qy: float, yaw: float, *, backward: bool,
                  vmax: float, omega_max: float, heading_gain: float) -> tuple[float, float]:
    """Paper §3.2: signed longitudinal and angular commands for differential drive."""
    magnitude = math.hypot(qx, qy)
    if magnitude <= 1e-6:
        return 0.0, 0.0
    sigma = -1.0 if backward else 1.0
    target_heading = math.atan2(sigma * qy, sigma * qx)
    heading_error = wrap_pi(target_heading - yaw)
    linear = sigma * min(max(0.0,vmax), magnitude * max(0.0,math.cos(heading_error)))
    angular = clamp(heading_gain * heading_error, -abs(omega_max), abs(omega_max))
    return linear, angular
