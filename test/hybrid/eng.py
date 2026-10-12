#!/usr/bin/env python3
"""Part 3 of the hybrid's go/no-go test (decision 11 Oct): the engine's own step responses (hybrid/engstep.js) against
the identified model and the physical core, both driven by the engine's recorded inputs (the road-wheel angle delta
and the hands' target p as they came through the driver's chain, the throttle), from zero deviation.

Fit per point and channel: 1 − ||prediction − engine|| / ||engine|| over the 0.5 s (1 = exact, 0 = no better than
predicting no change, < 0 worse). Medians over the points that held steadily (beta sd < 1.5 deg in the hold).

python3 hybrid/eng.py ENG.json [V=1]
"""
import json, sys
import numpy as np
from scipy.linalg import expm
D = json.load(open(sys.argv[1])); V = 'V=1' in sys.argv[2:]; dt = 1 / 60
def ident(p, U):   # U: per frame [d_delta, d_p, d_thr]
    Ad, Bd = np.array(p['Ad']), np.array(p['Bd']); x = np.zeros(3); out = [x.copy()]
    for k in range(30):
        x = Ad[:3, :3] @ x + Ad[:3, 5] * U[k][0] + Bd[:3, 0] * U[k][1] + Bd[:3, 1] * U[k][2]; out.append(x.copy())
    return np.array(out)
def phys(p, U):
    A, B, C = np.array(p['A']), np.array(p['B']), np.array(p['C']); n = A.shape[0]
    M = np.zeros((n + 2, n + 2)); M[:n, :n] = A; M[:n, n:] = B; E = expm(M * dt); Phi, Gam = E[:n, :n], E[:n, n:]
    x = np.zeros(n); out = [C @ x]
    for k in range(30):
        u = np.array([(U[k][0] + U[k + 1][0]) / 2, U[k][2]]); x = Phi @ x + Gam @ u; out.append(C @ x)   # delta: the frame's mean
    return np.array(out)
names = ['beta', 'r', 'v']; fits = {m: {e: {j: [] for j in range(3)} for e in ('steer', 'thr')} for m in ('id', 'ph')}
print(f"{D['drive']}: fit of the predicted response to the engine's (1 = exact) – identified | physics")
for p in sorted(D['pts'], key=lambda p: (p['R'], p['b'], p['ti'])):
    if 'A' not in p: print(f"  {p['R']}/{p['b']} t{p['ti']}: no steady slide in the model"); continue
    steady = p['sd'][0] * 57.3 < 1.5; line = f"  {p['R']:>3}/{p['b']:<2} t{p['ti']} β {p['mean'][0]*57.3:6.1f}±{p['sd'][0]*57.3:.1f}{'' if steady else ' (unsteady)'}"
    for e in ('steer', 'thr'):
        Y = np.array(p[e]); dY = Y[:, :3]; dthr = 0.05 if e == 'thr' else 0.0
        U = [[Y[k][5], Y[k][6], dthr] for k in range(31)]
        yi, yp = ident(p, U), phys(p, U)
        line += f" | {e}:"
        for j in range(3):
            nE = np.linalg.norm(dY[:, j]); fi = 1 - np.linalg.norm(yi[:, j] - dY[:, j]) / nE; fp = 1 - np.linalg.norm(yp[:, j] - dY[:, j]) / nE
            if steady: fits['id'][e][j].append(fi); fits['ph'][e][j].append(fp)
            line += f" {names[j]} {fi:+.2f}|{fp:+.2f}"
        if V: line += f" [eng@0.5s β{dY[-1,0]*57.3:+.2f}° r{dY[-1,1]*57.3:+.2f}°/s v{dY[-1,2]*3.6:+.2f} | id β{yi[-1,0]*57.3:+.2f} r{yi[-1,1]*57.3:+.2f} v{yi[-1,2]*3.6:+.2f} | ph β{yp[-1,0]*57.3:+.2f} r{yp[-1,1]*57.3:+.2f} v{yp[-1,2]*3.6:+.2f}]"
    print(line)
print("MEDIAN fit over the steady points (identified | physics):")
for e in ('steer', 'thr'):
    print(f"  {e:5} " + ' '.join(f"{names[j]} {np.median(fits['id'][e][j]):+.2f} | {np.median(fits['ph'][e][j]):+.2f}  " for j in range(3)) + f" (n {len(fits['id'][e][0])})")
