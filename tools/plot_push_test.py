import numpy as np
import matplotlib.pyplot as plt

run_a = "logs/push_sched_off"
run_b = "logs/push_sched_on"

ta, ea, ua = np.loadtxt(run_a, delimiter=",", comments="#", unpack=True)
tb, eb, ub = np.loadtxt(run_b, delimiter=",", comments="#", unpack=True)

fig, ax = plt.subplots(2, 1, sharex=True, figsize=(8, 5))

ax[0].plot(ta / 1000.0, np.degrees(ea), label="scheduling off")
ax[0].plot(tb / 1000.0, np.degrees(eb), label="scheduling on")
ax[0].axhline(0, lw=1.0, color="r", ls="--")
ax[0].set_ylabel("angle error (deg)")
ax[0].set_ylim(-12, 12)
ax[0].legend(loc="upper right", fontsize=8)
ax[0].grid(alpha=0.3)

ax[1].plot(ta / 1000.0, ua, label="scheduling off")
ax[1].plot(tb / 1000.0, ub, label="scheduling on")
ax[1].set_ylabel("u (N)")
ax[1].set_ylim(-12, 12)
ax[1].set_xlabel("time (s)")
ax[1].grid(alpha=0.3)

print(f"off: peak {np.abs(np.degrees(ea)).max():.2f} deg")
print(f"on:  peak {np.abs(np.degrees(eb)).max():.2f} deg")

plt.tight_layout()
plt.savefig("docs/results/stage3/push_test.png", dpi=150)
plt.show()

print(f"angle RMS  {np.sqrt(np.mean(e_deg**2)):.3f} deg")
print(f"angle peak {np.abs(e_deg).max():.3f} deg")
print(f"u RMS      {np.sqrt(np.mean(u**2)):.3f} N")
print(f"u peak     {np.abs(u).max():.3f} N  ({np.abs(u).max()/11.1*100:.1f}% of U_MAX)")