#!/usr/bin/env python3
"""Part 2 of the hybrid's go/no-go test (decision 11 Oct): the physical core's local dynamics (hybrid/dyn.js: A, B, C,
continuous, states vx vy r w0..w3 boost, inputs delta thr) against the identified models (Ad, Bd at 60 Hz, states
beta r v e h delta, inputs p thr) at the same points.

Compared: the responses of beta, r and v to a held step of the road-wheel angle (1 deg) and of the throttle (0.05),
after 1, 6, 15 and 30 frames (1/60 s); the identified model's slide rows with delta held as an input (p = delta: the
hands' target follows it). And the slowest poles (s, 1/s) of each.

python3 hybrid/dyn.py DYN.json [V=1]
"""
import json, sys
import numpy as np
from scipy.linalg import expm

D = json.load(open(sys.argv[1])); V = 'V=1' in sys.argv[2:]
dt = 1 / 60; K = [1, 6, 15, 30]; dd = 1 / 57.3; dthr = 0.05
def phys_resp(p, u):
    A, B, C = np.array(p['A']), np.array(p['B']), np.array(p['C']); n = A.shape[0]
    M = np.zeros((n + 2, n + 2)); M[:n, :n] = A; M[:n, n:] = B; E = expm(M * dt); Phi, Gam = E[:n, :n], E[:n, n:]
    x = np.zeros(n); out = []
    for k in range(1, 31):
        x = Phi @ x + Gam @ u
        out.append(C @ x)
    return np.array(out), np.linalg.eigvals(A)
def ident_resp(p, u):
    Ad, Bd = np.array(p['Ad']), np.array(p['Bd']); x = np.zeros(3); out = []
    for k in range(1, 31):
        x = Ad[:3, :3] @ x + Ad[:3, 5] * u[0] + Bd[:3, 0] * u[0] + Bd[:3, 1] * u[1]
        out.append(x.copy())
    ev = np.log(np.linalg.eigvals(Ad[:3, :3]).astype(complex)) / dt
    return np.array(out), ev
sc = np.array([57.3, 57.3, 3.6])   # beta deg, r deg/s, v km/h
names = ['beta', 'r', 'v']; rows = []
for p in D['pts']:
    R = {}
    for tag, u in (('d', np.array([dd, 0])), ('t', np.array([0, dthr]))):
        yp, evp = phys_resp(p, u); yi, evi = ident_resp(p, u)
        R[tag] = (yp * sc, yi * sc)
    slow_p = sorted([e for e in evp if abs(e) < 40], key=lambda e: -e.real)[:2]
    slow_i = sorted(evi, key=lambda e: -e.real)[:2]
    rows.append((p, R, slow_p, slow_i))
print(f"{D['drive']}: {len(rows)} points | step responses: identified → physics at {', '.join(f'{k} fr' for k in K)}")
agg = {(tag, j, k): [] for tag in 'dt' for j in range(3) for k in K}
for p, R, sp, si in sorted(rows, key=lambda r: (r[0]['R'], r[0]['b'], r[0]['ti'])):
    line = f"  {p['R']:>3}/{p['b']:<2} t{p['ti']} β{p['eq']['beta']*57.3:6.1f} r2 {' '.join(f'{x:.2f}' for x in p['r2'][:3])} |"
    for tag in 'dt':
        yp, yi = R[tag]
        for j in range(3):
            for k in K: agg[(tag, j, k)].append((yi[k - 1, j], yp[k - 1, j]))
        if V:
            line += f" {tag}:" + ' '.join(f"{names[j]} {yi[14, j]:+.2f}→{yp[14, j]:+.2f}" for j in range(3)) + ' |'
    line += f" poles id {' '.join(f'{e.real:+.2f}' for e in si)} ph {' '.join(f'{e.real:+.2f}' for e in sp)}"
    print(line)
print("per response (over the points): median identified, median physics, correlation over the points, median |phys-id|/|id|")
for tag, lab in (('d', 'steer +1°'), ('t', 'throttle +0.05')):
    for j in range(3):
        s = f"  {lab:15} → Δ{names[j]:4}"
        for k in K:
            a = np.array(agg[(tag, j, k)]); yi, yp = a[:, 0], a[:, 1]
            c = np.corrcoef(yi, yp)[0, 1] if np.std(yi) > 0 and np.std(yp) > 0 else float('nan')
            rel = np.median(np.abs(yp - yi) / np.maximum(np.abs(yi), 1e-3))
            s += f" | {k:>2}fr {np.median(yi):+7.3f} {np.median(yp):+7.3f} c{c:+.2f} e{rel:4.2f}"
        print(s)
