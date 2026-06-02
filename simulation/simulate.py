import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from plant import pendulum_dynamics, M, m, L, g, I, b
from controller import PIDController

# ─────────────────────────────────────────
# Simulation parameters
# ─────────────────────────────────────────
dt      = 0.01       # timestep (s) — matches controller sample rate
t_end   = 5.0        # simulation duration (s)
t_span  = (0, t_end)
t_eval  = np.arange(0, t_end, dt)

# ─────────────────────────────────────────
# Initial conditions
# ─────────────────────────────────────────
theta_0     = np.radians(5)   # X degree initial tilt
theta_dot_0 = 0.0             # starting from rest
x_0 = 0.0                     # cart starts at origin 
x_dot_0 = 0.0                 # cart starts from rest
state_0     = [theta_0, theta_dot_0, x_0, x_dot_0]

# ─────────────────────────────────────────
# Controller
# ─────────────────────────────────────────
pid = PIDController(
    Kp=50, Ki=1, Kd=5,
    dt=dt,
    output_limits=(-5.0, 5.0),
    setpoint=0.0
)

# ─────────────────────────────────────────
# Simulation loop
# ─────────────────────────────────────────
log = {
    't':     [],
    'theta': [],
    'x':     [],
    'x_dot': [],
    'u':     [],
    'P':     [],
    'I':     [],
    'D':     []
}

theta     = theta_0
theta_dot = theta_dot_0
x         = x_0
x_dot     = x_dot_0

for ti in t_eval:
    # 1. Controller sees current angle, produces torque
    u, terms = pid.compute(theta)

    # 2. Integrate plant one timestep forward
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

    # 3. Log everything
    log['t'].append(ti)
    log['theta'].append(np.degrees(theta))
    log['x'].append(x)
    log['x_dot'].append(x_dot)
    log['u'].append(u)
    log['P'].append(terms['P'])
    log['I'].append(terms['I'])
    log['D'].append(terms['D'])

# ─────────────────────────────────────────
# Plots
# ─────────────────────────────────────────
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 7))

ax1.plot(log['t'], log['theta'], label='Tilt angle')
ax1.axhline(0, color='r', linestyle='--', label='Setpoint')
ax1.set_ylabel('Angle (degrees)')
ax1.set_title('Closed-Loop PID Response')
ax1.legend()
ax1.grid(True)

ax2.plot(log['t'], log['P'], label='P term')
ax2.plot(log['t'], log['I'], label='I term')
ax2.plot(log['t'], log['D'], label='D term')
ax2.plot(log['t'], log['u'], label='Total output', linewidth=2)
ax2.set_ylabel('Torque (N.m)')
ax2.legend()
ax2.grid(True)

ax3.plot(log['t'], log['x'], label='Cart position')
ax3.set_ylabel('Position (m)')
ax3.set_xlabel('Time (s)')
ax3.legend()
ax3.grid(True)

plt.tight_layout()
plt.show()