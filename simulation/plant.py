import numpy as np

"""
Inverted pendulum plant model — Stage 1 full coupled cart-pole model.
State vector: x = [theta, theta_dot, x, x_dot]
  theta     — pendulum angle from vertical (rad), positive = forward lean
  theta_dot — angular velocity (rad/s)
  x         — cart position (m)
  x_dot     — cart velocity (m/s)

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
    theta, theta_dot, x, x_dot = state
    denominator = (M + m) * (I + m * L**2) - (m * L * np.cos(theta))**2
    x_ddot = ((I + m * L**2) * (u + m * L * theta_dot**2 * np.sin(theta)) + m * L * np.cos(theta) * (m * g * L * np.sin(theta) + b * theta_dot - u)) / denominator
    theta_ddot = (m * L * np.cos(theta) * (u + m * L * theta_dot**2 * np.sin(theta)) + (M + m) * (m * g * L * np.sin(theta) + b * theta_dot - u)) / denominator
    return [theta_dot, theta_ddot, x_dot, x_ddot]

def linearised_matrices():
    denom = (M + m) * (I + m * L**2) - (m * L)**2

    A = np.array([
        [0,                          1,  0,  0],
        [m**2 * g * L**2 / denom,   -b * (M + m) / denom,  0,  0],
        [0,                          0,  0,  1],
        [-m * g * L * (M + m) / denom,  b * m * L / denom,  0,  0]
    ])

    B = np.array([
        [0],
        [-m * L / denom],
        [0],
        [(I + m * L**2) / denom]
    ])

    return A, B