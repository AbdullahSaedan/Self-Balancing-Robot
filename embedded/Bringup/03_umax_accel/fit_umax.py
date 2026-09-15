"""
fit_umax.py — estimate U_MAX from 03_umax_accel runs.

Usage:  python fit_umax.py umax_runs.txt

Paste all Serial Monitor output (any number of runs) into umax_runs.txt.
Fits x(t) = vmax * (s - tau*(1 - exp(-s/tau))),  s = t - td,
the position form of a first-order velocity step. Initial acceleration
a0 = vmax / tau, so U_MAX = M_TOTAL * a0.
"""
import sys
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# ===================== CONSTANTS =====================
M_TOTAL        = 0.621     # kg — REWEIGH the rebuilt robot and update this
WHEEL_R        = 0.0325    # m
COUNTS_PER_REV = 1000
M_PER_COUNT    = 2 * np.pi * WHEEL_R / COUNTS_PER_REV

# ===================== PARSE LOG =====================
def parse(path):
    runs, cur, d = [], None, None
    for line in open(path):
        line = line.strip()
        if line.startswith("# dir"):
            d, cur = line.split()[-1], []
        elif line.startswith("# end") and cur is not None:
            runs.append((d, np.array(cur, dtype=float)))
            cur = None
        elif cur is not None and line and line[0].isdigit():
            cur.append([float(v) for v in line.split(",")])
    return runs

# ===================== MODEL =====================
def model(t, vmax, tau, td):
    s = np.clip(t - td, 0, None)
    return vmax * (s - tau * (1 - np.exp(-s / tau)))

# ===================== FIT EACH RUN =====================
def fit_run(data):
    t = data[:, 0] / 1000.0
    counts = 0.5 * (data[:, 1] + data[:, 2])       # average of both wheels
    x = np.abs(counts) * M_PER_COUNT               # direction doesn't matter here
    p, _ = curve_fit(model, t, x, p0=[0.45, 0.02, 0.002],
                     bounds=([0.05, 0.001, 0.0], [3.0, 0.5, 0.03]))
    return t, x, p

def main(path):
    runs = parse(path)
    if not runs:
        sys.exit("No runs found — check the file contains '# dir' ... '# end' blocks.")
    results = {"f": [], "b": []}
    fig, ax = plt.subplots(figsize=(7, 4))
    for i, (d, data) in enumerate(runs):
        t, x, (vmax, tau, td) = fit_run(data)
        a0 = vmax / tau
        results[d].append(M_TOTAL * a0)
        print(f"run {i+1} ({d}): vmax={vmax:.3f} m/s  tau={tau*1000:.1f} ms  "
              f"td={td*1000:.1f} ms  a0={a0:.2f} m/s^2  U_MAX={M_TOTAL*a0:.2f} N")
        ax.plot(t * 1000, x * 1000, ".", ms=2)
        ax.plot(t * 1000, model(t, vmax, tau, td) * 1000, "-", lw=1)
    print()
    for d, u in results.items():
        if u:
            print(f"{d}: U_MAX = {np.mean(u):.2f} ± {np.std(u):.2f} N  (n={len(u)})")
    allu = results["f"] + results["b"]
    print(f"\nOverall U_MAX = {np.mean(allu):.2f} ± {np.std(allu):.2f} N  "
          f"(firmware currently 2.77 N)")
    ax.set_xlabel("time (ms)"); ax.set_ylabel("position (mm)")
    ax.set_title("Full-duty acceleration from rest: data (dots) and fit (lines)")
    ax.grid(alpha=0.3); fig.tight_layout()
    fig.savefig("umax_fit.png", dpi=150)
    print("Saved umax_fit.png")

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "umax_runs.txt")
