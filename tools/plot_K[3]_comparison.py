import numpy as np
import matplotlib.pyplot as plt

run_a = "logs/k3_16"
run_b = "logs/k3_27"

ta, ea, ua = np.loadtxt(run_a, delimiter=",", comments="#", unpack=True)
tb, eb, ub = np.loadtxt(run_b, delimiter=",", comments="#", unpack=True)

fig, ax = plt.subplots(2, 1, sharex=True, sharey=False, figsize=(8, 5))

ax[0].plot(ta / 1000.0, np.degrees(ea), label="K[3] = 16")
ax[0].plot(tb / 1000.0, np.degrees(eb), label="K[3] = 27", alpha=0.8)
ax[0].axhline(0, lw=1.0, color="r", ls="--")
ax[0].set_ylabel("angle error (deg)")
ax[0].legend(loc="upper right", fontsize=8)
ax[0].grid(alpha=0.3)

ax[1].plot(ta / 1000.0, ua, label="K[3] = 16")
ax[1].plot(tb / 1000.0, ub, label="K[3] = 27", alpha=0.8)
ax[1].set_ylabel("u (N)")
ax[1].set_xlabel("time (s)")
ax[1].grid(alpha=0.3)

plt.tight_layout()
plt.savefig("docs/results/stage3/k3_comparison.png", dpi=150)
plt.show()

for name, e, u in [("K[3] = 16", ea, ua), ("K[3] = 27", eb, ub)]:
    print(f"{name}")
    print(f"  angle RMS  {np.sqrt(np.mean(np.degrees(e)**2)):.3f} deg")
    print(f"  angle peak {np.abs(np.degrees(e)).max():.3f} deg")
    print(f"  u RMS      {np.sqrt(np.mean(u**2)):.3f} N")