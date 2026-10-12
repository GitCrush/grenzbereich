#!/usr/bin/env python3
"""The model the controller would get (decision 11 Oct): the identification's form (60 Hz, rows beta r v from
[beta r v delta thr], the steering row delta from [beta r v delta p], as DC_MASK) fitted to the physical core's own
simulated responses (the core with the steering: inputs p thr) – against the engine's step responses, driven by the
engine's recorded hands' target p and throttle; beside it the full core with steering and the identified model.
python3 hybrid/ctlmodel.py ENG.json [AMP=1] (AMP: the fit's steering excitation in degrees)"""
import json, sys
import numpy as np
from scipy.linalg import expm
D = json.load(open(sys.argv[1])); dt = 1 / 60
AMP = float(next((a[4:] for a in sys.argv[2:] if a.startswith('AMP=')), 1.0))
DROW = next((a[5:] for a in sys.argv[2:] if a.startswith('DROW=')), 'full')
TR = int(next((a[2:] for a in sys.argv[2:] if a.startswith('T=')), 30)); NR = 6000 // TR
def disc(A, B):
    n = A.shape[0]; m = B.shape[1]; M = np.zeros((n + m, n + m)); M[:n, :n] = A; M[:n, n:] = B; E = expm(M * dt); return E[:n, :n], E[:n, n:]
def fitG6(A, B, C, seed=1):
    rng = np.random.default_rng(seed); Phi, Gam = disc(A, B); rows = []
    for run in range(NR):
        x = np.zeros(A.shape[0]); u = np.zeros(2)
        for k in range(TR):
            if k % 6 == 0: u[0] = (rng.random() - 0.5) * 2 * AMP / 57.3
            if k % 15 == 0: u[1] = (rng.random() - 0.5) * 0.1
            y = C @ x; x = Phi @ x + Gam @ u; rows.append((y, u.copy(), C @ x))
    Y0 = np.array([r[0] for r in rows]); U = np.array([r[1] for r in rows]); Y1 = np.array([r[2] for r in rows])
    F = np.zeros((4, 4)); G = np.zeros((4, 2))
    Xv = np.c_[Y0, U[:, 1]]                       # beta r v delta | thr
    for j in range(3): th = np.linalg.lstsq(Xv, Y1[:, j], rcond=None)[0]; F[j] = th[:4]; G[j, 1] = th[4]
    if DROW == 'full':
        Xd = np.c_[Y0, U[:, 0]]                       # beta r v delta | p
        th = np.linalg.lstsq(Xd, Y1[:, 3], rcond=None)[0]; F[3] = th[:4]; G[3, 0] = th[4]
    else:                                              # delta | p only: the steering's own lag
        Xd = np.c_[Y0[:, 3], U[:, 0]]
        th = np.linalg.lstsq(Xd, Y1[:, 3], rcond=None)[0]; F[3, 3] = th[0]; G[3, 0] = th[1]
    return F, G
def run_d(F, G, U):
    y = np.zeros(4); out = [y.copy()]
    for k in range(30): y = F @ y + G @ U[k]; out.append(y.copy())
    return np.array(out)
def run_c(A, B, C, U):
    Phi, Gam = disc(A, B); x = np.zeros(A.shape[0]); out = [C @ x]
    for k in range(30): x = Phi @ x + Gam @ U[k]; out.append(C @ x)
    return np.array(out)
names = ['beta', 'r', 'v', 'delta']; V = ['id', 'core', 'G6']; fits = {v: {e: {j: [] for j in range(4)} for e in ('steer', 'thr')} for v in V}
for p in D['pts']:
    if 'As' not in p or p['sd'][0] * 57.3 >= 0.5 or p['sd'][5] * 57.3 >= 1: continue
    As, Bs, Cs = np.array(p['As']), np.array(p['Bs']), np.array(p['Cs']); F, G = fitG6(As, Bs, Cs)
    Ad, Bd = np.array(p['Ad']), np.array(p['Bd']); ix = [0, 1, 2, 5]; Fi = Ad[np.ix_(ix, ix)]; Gi = Bd[ix]
    for e in ('steer', 'thr'):
        Y = np.array(p[e]); dY = Y[:, [0, 1, 2, 5]]; U = [np.array([Y[k][6], 0.05 if e == 'thr' else 0.0]) for k in range(31)]
        ys = {'id': run_d(Fi, Gi, U), 'core': run_c(As, Bs, Cs, U), 'G6': run_d(F, G, U)}
        for v in V:
            for j in range(4):
                nE = np.linalg.norm(dY[:, j])
                if nE > 1e-9: fits[v][e][j].append(1 - np.linalg.norm(ys[v][:, j] - dY[:, j]) / nE)
n = len(fits['id']['steer'][0])
print(f"{D['drive']}: {n} steady points, inputs p and throttle as recorded (steering excitation of the fit ±{AMP}°) – median fit (1 = exact):")
for v in V:
    print(f"  {v:5} " + ' | '.join(f"{e} " + ' '.join(f"{names[j]} {np.median(fits[v][e][j]):+.2f}" for j in range(4) if fits[v][e][j]) for e in ('steer', 'thr')))
