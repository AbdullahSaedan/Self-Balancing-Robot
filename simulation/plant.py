import numpy as np

"""
Inverted pendulum plant model.
State vector: x = [theta, theta_dot]
  theta     — pendulum angle from vertical (rad), positive = forward lean
  theta_dot — angular velocity (rad/s)

This is the linearised model valid for small angles (|theta| < ~15 deg).
We use the nonlinear model for simulation and the linear model for control design.
"""

# Physical parameters — tune these to match eventual hardware
M = 0.5      # wheel/cart mass (kg)
m = 1.0      # pendulum (body) mass (kg)
L = 0.15     # distance from pivot to centre of mass (m)
g = 9.81     # gravity (m/s^2)
I = m * L**2 # moment of inertia (approximation for rod, kg.m^2)
b = 0.05     # viscous damping at pivot (N.m.s/rad)

def pendulum_dynamics(t, state, u):
    theta, theta_dot = state
    denominator = I + m * L**2
    theta_ddot = (m * g * L * np.sin(theta) - b * theta_dot + u) / denominator
    return [theta_dot, theta_ddot]

def linearised_matrices():
    denom = I + m * L**2
    A = np.array([
        [0,                  1],
        [m * g * L / denom,  -b / denom]
    ])
    B = np.array([
        [0        ],
        [1 / denom]
    ])
    return A, B