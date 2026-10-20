"""LQR gain design for the balancing controller."""

import numpy as np
from scipy.linalg import solve_discrete_are

from cubli_control.model import CubliParams, linearized


def discretize(a, b, dt: float):
    """Zero-order-hold discretisation via the augmented matrix exponential."""
    from scipy.linalg import expm

    n, m = b.shape
    aug = np.zeros((n + m, n + m))
    aug[:n, :n] = a
    aug[:n, n:] = b
    phi = expm(aug * dt)
    return phi[:n, :n], phi[:n, n:]


def lqr_gain(p: CubliParams, dt: float, q_diag, r: float, delay_steps: int = 0) -> np.ndarray:
    """Discrete LQR gain for u_k = -K [theta_b, dtheta_b, dtheta_w, u_{k-d}, ..., u_{k-1}].

    With delay_steps = d > 0 the design assumes each command takes effect d control
    steps late (sensing + transport + actuation), and K gets d extra columns that act on
    the d previous commands still "in flight". With d = 0 it is the plain 1x3 LQR gain.
    """
    a, b = linearized(p)
    ad, bd = discretize(a, b, dt)
    n, d = ad.shape[0], int(delay_steps)
    if d > 0:
        # State [z, u_{k-d}, ..., u_{k-1}]: z is driven by the oldest command, the rest shift.
        size = n + d
        a_aug = np.zeros((size, size))
        a_aug[:n, :n] = ad
        a_aug[:n, n:n + 1] = bd
        a_aug[n:size - 1, n + 1:size] = np.eye(d - 1)
        b_aug = np.zeros((size, 1))
        b_aug[size - 1, 0] = 1.0
        ad, bd = a_aug, b_aug
    q = np.zeros((ad.shape[0], ad.shape[0]))
    q[:n, :n] = np.diag(q_diag)
    rm = np.array([[r]])
    s = solve_discrete_are(ad, bd, q, rm)
    return np.linalg.solve(rm + bd.T @ s @ bd, bd.T @ s @ ad)
