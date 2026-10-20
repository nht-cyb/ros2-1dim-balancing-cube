"""1D Cubli dynamics (cube balancing on an edge with one reaction wheel).

Model and default parameters follow Gajamohan et al., "The Cubli: A Cube that can
Jump Up and Balance" (IROS 2012).

State  x = [theta_b, dtheta_b, theta_w, dtheta_w]
  theta_b  : body tilt about the edge [rad], 0 = centre of mass above the edge
  theta_w  : wheel angle relative to the body [rad]
Input  u = T_m, motor torque applied to the wheel (reaction -T_m on the body) [N m]
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CubliParams:
    m_b: float = 0.419       # body mass (without wheel) [kg]
    m_w: float = 0.204       # wheel mass [kg]
    l: float = 0.085         # pivot -> wheel centre [m]
    l_b: float = 0.075       # pivot -> body centre of mass [m]
    theta_b: float = 3.34e-3  # body inertia about the pivot [kg m^2]
    theta_w: float = 0.57e-3  # wheel inertia about its axis [kg m^2]
    c_b: float = 1.02e-3     # body (pivot) friction [N m s/rad]
    c_w: float = 0.05e-3     # wheel bearing friction [N m s/rad]
    g: float = 9.81
    max_torque: float = 0.25  # motor torque limit [N m]

    @property
    def inertia_body(self) -> float:
        """Body + wheel point mass inertia about the pivot."""
        return self.theta_b + self.m_w * self.l ** 2

    @property
    def mass_moment(self) -> float:
        """m_b * l_b + m_w * l, multiplies g * sin(theta_b)."""
        return self.m_b * self.l_b + self.m_w * self.l


FALL_ANGLE = math.pi / 4  # cube lies flat on a face at +/-45 deg


def dynamics(x, torque: float, p: CubliParams) -> np.ndarray:
    """Return dx/dt for the nonlinear model, including the floor holding a fallen cube."""
    th_b, dth_b, _, dth_w = x
    i_b = p.inertia_body
    gravity = p.mass_moment * p.g * math.sin(th_b)
    wheel_drive = torque - p.c_w * dth_w

    ddth_b = (gravity - wheel_drive - p.c_b * dth_b) / i_b
    on_floor = abs(th_b) >= FALL_ANGLE - 1e-6 and th_b * dth_b >= 0.0 and th_b * ddth_b > 0.0
    if on_floor:
        # Lying on a face and pressed into the floor: the body stays put,
        # only the wheel moves.
        return np.array([0.0, 0.0, dth_w, wheel_drive / p.theta_w])
    ddth_w = (wheel_drive * (i_b + p.theta_w) / (p.theta_w * i_b)
              - (gravity - p.c_b * dth_b) / i_b)
    return np.array([dth_b, ddth_b, dth_w, ddth_w])


def rk4_step(x, torque: float, dt: float, p: CubliParams) -> np.ndarray:
    k1 = dynamics(x, torque, p)
    k2 = dynamics(x + 0.5 * dt * k1, torque, p)
    k3 = dynamics(x + 0.5 * dt * k2, torque, p)
    k4 = dynamics(x + dt * k3, torque, p)
    return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def brake(x, p: CubliParams, efficiency: float = 1.0) -> np.ndarray:
    """Lock the wheel to the body instantly (the jump-up brake).

    The brake is internal and the floor reaction passes through the pivot edge, so
    angular momentum about the edge is conserved; `efficiency` < 1 models a lossy brake.
    """
    x = np.array(x, dtype=float)
    momentum = p.inertia_body * x[1] + p.theta_w * (x[1] + x[3])
    x[1] = efficiency * momentum / (p.inertia_body + p.theta_w)
    x[3] = 0.0
    return x


def jump_wheel_speed(p: CubliParams, start_angle: float = FALL_ANGLE) -> float:
    """Wheel speed that, after braking, lifts the cube from start_angle exactly to upright."""
    lift_energy = p.mass_moment * p.g * (1.0 - math.cos(start_angle))
    total_inertia = p.inertia_body + p.theta_w
    return math.sqrt(2.0 * lift_energy * total_inertia) / p.theta_w


def apply_ground_contact(x) -> np.ndarray:
    """Stop the body when it lands on a face (inelastic contact)."""
    x = np.array(x, dtype=float)
    if abs(x[0]) >= FALL_ANGLE:
        x[0] = math.copysign(FALL_ANGLE, x[0])
        if x[0] * x[1] > 0:
            x[1] = 0.0
    return x


def linearized(p: CubliParams):
    """Linearisation about the upright equilibrium.

    Reduced state z = [theta_b, dtheta_b, dtheta_w] (the wheel angle itself
    does not affect the dynamics, so it is left out).
    """
    i_b = p.inertia_body
    mg = p.mass_moment * p.g
    k_w = (i_b + p.theta_w) / (p.theta_w * i_b)
    a = np.array([
        [0.0, 1.0, 0.0],
        [mg / i_b, -p.c_b / i_b, p.c_w / i_b],
        [-mg / i_b, p.c_b / i_b, -p.c_w * k_w],
    ])
    b = np.array([[0.0], [-1.0 / i_b], [k_w]])
    return a, b
