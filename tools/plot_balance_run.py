import numpy as np
import matplotlib.pyplot as plt

path = "logs/balance_run"
t_ms, e, u = np.loadtxt(path, delimiter=",", comments="#", unpack=True)

t = t_ms / 1000.0
e_deg = np.degrees(e)

fig, ax = plt.subplots(2, 1, sharex=True, figsize=(8, 5))

ax[0].plot(t, e_deg)
ax[0].axhline(0, lw=1.0, color="r", ls="--", label="setpoint")
ax[0].legend(loc="upper right", fontsize=8)
ax[0].set_ylabel("angle error (deg)")
ax[0].grid(alpha=0.3)

ax[1].plot(t, u)
ax[1].set_ylabel("u (N)")
ax[1].set_xlabel("time (s)")
ax[1].grid(alpha=0.3)

plt.tight_layout()
plt.savefig("docs/results/stage3/balance_run.png", dpi=150)
plt.show()

print(f"angle RMS  {np.sqrt(np.mean(e_deg**2)):.3f} deg")
print(f"angle peak {np.abs(e_deg).max():.3f} deg")
print(f"u RMS      {np.sqrt(np.mean(u**2)):.3f} N")
print(f"u peak     {np.abs(u).max():.3f} N  ({np.abs(u).max()/11.1*100:.1f}% of U_MAX)")