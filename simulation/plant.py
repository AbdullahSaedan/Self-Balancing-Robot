import numpy as np

"""
Inverted pendulum plant model — Stage 1 full coupled cart-pole model.
State vector: x = [theta, theta_dot, x, x_dot]
  theta     — pendulum angle from vertical (rad), positive = forward lean
  theta_dot — angular velocity (rad/s)
  x         — cart position (m)
  x_dot     — cart velocity (m/s)

"""

# Physical parameters — tune these to match eventual hardware
M = 0.056      # wheel mass (kg)
m = 0.565      # pendulum (body) mass (kg)
L = 0.043    # distance from pivot to centre of mass (m)
g = 9.81     # gravity (m/s^2)
I = 0.00082 # moment of inertia (approximation for rod, kg.m^2)
b = 0.0005     # viscous damping at pivot (N.m.s/rad)
b_drive = 26.0 # motor back-EMF drag on the cart (N.s/m), from U_MAX / v_max; 0 = Stage 1 model

def pendulum_dynamics(t, state, u):
    theta, theta_dot, x, x_dot = state
    F = u - b_drive * x_dot
    denominator = (M + m) * (I + m * L**2) - (m * L * np.cos(theta))**2
    x_ddot = ((I + m * L**2) * (F - m * L * theta_dot**2 * np.sin(theta))
              + m * L * np.cos(theta) * (m * g * L * np.sin(theta) - b * theta_dot)) / denominator
    theta_ddot = (m * L * np.cos(theta) * (F - m * L * theta_dot**2 * np.sin(theta))
                  + (M + m) * (m * g * L * np.sin(theta) - b * theta_dot)) / denominator
    return [theta_dot, theta_ddot, x_dot, x_ddot]

def linearised_matrices():
    denom = (M + m) * (I + m * L**2) - (m * L)**2

    A = np.array([
        [0,                        1,                 0,  0],
        [(M + m) * m * g * L / denom,  -(M + m) * b / denom,  0,  -m * L * b_drive / denom],
        [0,                        0,                 0,  1],
        [m**2 * g * L**2 / denom,     -m * L * b / denom,     0,  -(I + m * L**2) * b_drive / denom]
    ])

    B = np.array([
        [0],
        [m * L / denom],
        [0],
        [(I + m * L**2) / denom]
    ])

    return A, B