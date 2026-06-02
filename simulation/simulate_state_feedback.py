import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from plant import pendulum_dynamics, M, m, L, g, I, b

# ─────────────────────────────────────────
# Simulation parameters
# ─────────────────────────────────────────
dt    = 0.01
t_end = 5.0
t_eval = np.arange(0, t_end, dt)

# ─────────────────────────────────────────
# Initial conditions
# ─────────────────────────────────────────
theta_0     = np.radians(5)
theta_dot_0 = 0.0
x_0         = 0.0
x_dot_0     = 0.0

# ─────────────────────────────────────────
# State feedback gains from pole placement
# Computed analytically using linearised_matrices()
# Desired poles: [-3, -4, -5, -6]
# ─────────────────────────────────────────
K = np.array([35.7770091, 4.25784006, 2.26993285, 2.14053313])

# ─────────────────────────────────────────
# Simulation loop
# ─────────────────────────────────────────
log = {
    't':     [],
    'theta': [],
    'x':     [],
    'u':     []
}

theta     = theta_0
theta_dot = theta_dot_0
x         = x_0
x_dot     = x_dot_0

for ti in t_eval:
    # State feedback control law
    state = np.array([theta, theta_dot, x, x_dot])
    u = float(-(K @ state))
    u = np.clip(u, -20.0, 20.0)

    # Integrate plant one timestep forward
    sol = solve_ivp(
        pendulum_dynamics,
        [ti, ti + dt],
        [theta, theta_dot, x, x_dot],
        args=(u,),
        max_step=dt
    )

    theta     = sol.y[0][-1]
    theta_dot = sol.y[1][-1]
    x         = sol.y[2][-1]
    x_dot     = sol.y[3][-1]

    log['t'].append(ti)
    log['theta'].append(np.degrees(theta))
    log['x'].append(x)
    log['u'].append(u)

# ─────────────────────────────────────────
# Plots
# ─────────────────────────────────────────
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 7))

ax1.plot(log['t'], log['theta'], label='Tilt angle')
ax1.axhline(0, color='r', linestyle='--', label='Setpoint')
ax1.set_ylabel('Angle (degrees)')
ax1.set_title('State Feedback Control — Pole Placement')
ax1.legend()
ax1.grid(True)

ax2.plot(log['t'], log['u'], label='Control output', color='red')
ax2.set_ylabel('Torque (N.m)')
ax2.legend()
ax2.grid(True)

ax3.plot(log['t'], log['x'], label='Cart position')
ax3.set_ylabel('Position (m)')
ax3.set_xlabel('Time (s)')
ax3.legend()
ax3.grid(True)

plt.tight_layout()
plt.savefig('docs/results/stage1/state_feedback_pole_placement.png', dpi=150, bbox_inches='tight')
plt.show()