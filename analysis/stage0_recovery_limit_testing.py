import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from simulation.plant import pendulum_dynamics
from simulation.controller import PIDController

def angle_sweep():
    dt     = 0.01
    t_end  = 5.0
    t_eval = np.arange(0, t_end, dt)

    initial_angles = range(5, 50, 5)
    failure_threshold = np.radians(45)

    plt.figure(figsize=(10, 6))

    for angle_deg in initial_angles:
        theta     = np.radians(angle_deg)
        theta_dot = 0.0

        # Reset controller state for each run
        pid = PIDController(Kp=50, Ki=1, Kd=5, dt=dt,
                            output_limits=(-5.0, 5.0), setpoint=0.0)

        theta_log = []
        failed    = False

        for ti in t_eval:
            u, _ = pid.compute(theta)

            sol = solve_ivp(
                pendulum_dynamics,
                [ti, ti + dt],
                [theta, theta_dot],
                args=(u,),
                max_step=dt
            )

            theta     = sol.y[0][-1]
            theta_dot = sol.y[1][-1]
            theta_log.append(np.degrees(theta))

            # Check failure condition - early exit if so
            if abs(theta) > failure_threshold:
                failed = True
                break

        label = f"{angle_deg}° — {'FAILED' if failed else 'RECOVERED'}"
        # Plot only the portion of the log that was simulated (up to failure if it occurred)
        plt.plot(t_eval[:len(theta_log)], theta_log, label=label)

    plt.axhline( 45, color='r', linestyle='--', linewidth=1)
    plt.axhline(-45, color='r', linestyle='--', linewidth=1, label='Failure threshold')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.title('Controller Recovery Limit — Varying Initial Theta')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    #plt.savefig('docs/results/angle_sweep_1nm_limits.png', dpi=150, bbox_inches='tight')
    plt.show()

angle_sweep()