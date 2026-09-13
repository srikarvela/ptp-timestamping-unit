#!/usr/bin/env python3
"""Diff the RTL replay of the servo against the MATLAB golden vectors.

    python3 python/servo_diff.py            (after `make matlab` and `make sim-servo_golden`)

Reads  matlab/vectors/servo_vectors.txt     golden per-step clock state
       matlab/vectors/servo_trajectory.csv  golden offsets (script + Simulink)
       build/servo_rtl_trace.txt            RTL per-step clock state
and reports bit-exactness, lock time and residual statistics.  Writes
docs/servo_rtl_vs_golden.png if matplotlib is available.
"""

from __future__ import annotations

import csv
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VEC = os.path.join(ROOT, "matlab", "vectors", "servo_vectors.txt")
TRAJ = os.path.join(ROOT, "matlab", "vectors", "servo_trajectory.csv")
TRACE = os.path.join(ROOT, "build", "servo_rtl_trace.txt")


def read_vectors(path):
    with open(path) as f:
        toks = f.read().split()
    k = int(toks[0], 16)
    rows = []
    for i in range(k):
        t = toks[1 + 9 * i: 10 + 9 * i]
        rows.append(dict(F=int(t[0], 16), adj=int(t[1], 16), N=int(t[4], 16),
                         sec=int(t[5], 16), ns=int(t[6], 16), frac=int(t[7], 16),
                         master=int(t[8], 16)))
    return rows


def read_trace(path):
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            k, sec, ns, frac, off = line.split()
            rows.append(dict(k=int(k), sec=int(sec), ns=int(ns), frac=int(frac), offset=float(off)))
    return rows


def main() -> int:
    for p in (VEC, TRACE):
        if not os.path.exists(p):
            print(f"missing {p}")
            return 2
    gold = read_vectors(VEC)
    rtl = read_trace(TRACE)
    n = min(len(gold), len(rtl))
    mism = [k for k in range(n) if (gold[k]["sec"], gold[k]["ns"], gold[k]["frac"])
            != (rtl[k]["sec"], rtl[k]["ns"], rtl[k]["frac"])]
    print(f"golden steps: {len(gold)}   rtl steps: {len(rtl)}   compared: {n}")
    print(f"bit-exact mismatches (sec, ns, frac): {len(mism)}")
    for k in mism[:5]:
        g, r = gold[k], rtl[k]
        print(f"  step {k}: golden {g['sec']}.{g['ns']:09d}.{g['frac']:08x}  rtl {r['sec']}.{r['ns']:09d}.{r['frac']:08x}")

    off = [r["offset"] for r in rtl[:n]]
    thr = 100.0
    steps = [g["N"] for g in gold[:n]]
    ts_ms = steps[0] * 6.4e-6
    phase_steps = [k for k in range(n) if gold[k]["adj"]]
    # lock: last excursion beyond thr before the midpoint disturbance
    half = n // 2
    bad = [k for k in range(half) if abs(off[k]) >= thr]
    k_lock = (bad[-1] + 1) if bad else 0
    seg = off[k_lock:half]
    rms = math.sqrt(sum(x * x for x in seg) / len(seg)) if seg else float("nan")
    peak = max((abs(x) for x in seg), default=float("nan"))
    seg2 = off[half:]
    bad2 = [k for k in range(len(seg2)) if abs(seg2[k]) >= thr]
    print(f"phase steps applied at: {phase_steps}")
    if seg:
        print(f"locked (|offset| < {thr:.0f} ns) from step {k_lock} = {k_lock * ts_ms:.0f} ms; "
              f"residual RMS {rms:.1f} ns, peak {peak:.1f} ns")
    else:
        print(f"never locked to |offset| < {thr:.0f} ns before step {half}")
    if seg2:
        print(f"after the drift step at step {half}: peak {max(abs(x) for x in seg2):.0f} ns, "
              f"re-locked after {(bad2[-1] + 1) if bad2 else 0} steps")
    print(f"final offset {off[-1]:+.3f} ns   final FREQ_ADJ {gold[-1]['F'] - (1 << 32) if gold[-1]['F'] >> 31 else gold[-1]['F']} LSB")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        sim = None
        if os.path.exists(TRAJ):
            with open(TRAJ) as f:
                sim = [float(r["offset_simulink_ns"]) for r in csv.DictReader(f)]
        # RTL offset is sampled at the END of step k; Simulink's at the start
        t = [(k + 1) * ts_ms for k in range(n)]
        t_sim = [k * ts_ms for k in range(n)]
        fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        ax[0].plot(t, off, "b-", lw=1.2, label="RTL (Icarus replay)")
        if sim:
            ax[0].plot(t_sim[:len(sim)], sim[:n], "r--", lw=0.9, label="Simulink")
        ax[0].axhline(0, color="k", ls=":")
        ax[0].set_ylabel("offset to master (ns)")
        ax[0].legend(); ax[0].grid(True)
        ax[0].set_title(f"RTL vs golden: {n} steps, {len(mism)} mismatches; lock at {k_lock * ts_ms:.0f} ms, residual RMS {rms:.1f} ns")
        ax[1].plot(t, off, "b-", lw=1.2)
        ax[1].set_ylim(-250, 250); ax[1].axhline(0, color="k", ls=":")
        ax[1].set_ylabel("offset (ns), zoom"); ax[1].set_xlabel("time (ms)"); ax[1].grid(True)
        out = os.path.join(ROOT, "docs", "servo_rtl_vs_golden.png")
        fig.tight_layout(); fig.savefig(out, dpi=110)
        print(f"wrote {os.path.relpath(out, ROOT)}")
    except ImportError:
        print("(matplotlib not installed - no plot)")

    return 0 if not mism and n == len(gold) else 1


if __name__ == "__main__":
    sys.exit(main())
