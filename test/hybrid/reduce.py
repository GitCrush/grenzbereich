#!/usr/bin/env python3
"""Which reduction of the physical core fits the controller's state basis (beta r v e h delta, 60 Hz)? (decision 11 Oct)
Against the engine's step responses (hybrid/engstep.js): the full core (states vx vy r w0..w3 boost) and reductions –
W: the wheels residualised (quasi-steady), boost kept; WB: wheels and boost residualised; G: a 3-state 60 Hz model
(beta r v, inputs delta thr) fitted by least squares to the core's own simulated responses (open loop, short runs from
the steady slide, random steering and throttle steps) – the identification's form without its closed loop and noise.
Prints the slow eigenvalues of the core and the median fits (1 = exact) per variant.
python3 hybrid/reduce.py ENG.json"""
import json, sys
import numpy as np
from scipy.linalg import expm
D = json.load(open(sys.argv[1])); dt = 1 / 60; rng = np.random.default_rng(1)
def disc(A, B):
    n = A.shape[0]; M = np.zeros((n + B.shape[1], n + B.shape[1])); M[:n, :n] = A; M[:n, n:] = B; E = expm(M * dt); return E[:n, :n], E[:n, n:]
def resid(A, B, C, fast):
    s = [i for i in range(A.shape[0]) if i not in fast]; f = fast
    Aff = A[np.ix_(f, f)]; X = np.linalg.solve(Aff, A[np.ix_(f, s)]); Y = np.linalg.solve(Aff, B[f])
    return A[np.ix_(s, s)] - A[np.ix_(s, f)] @ X, B[s] - A[np.ix_(s, f)] @ Y, C[:, s] - C[:, f] @ X, -C[:, f] @ Y
def sim_ct(A, B, C, Dm, U):
    Phi, Gam = disc(A, B); x = np.zeros(A.shape[0]); out = [np.zeros(3)]
    for k in range(30):
        u = np.array([(U[k][0] + U[k + 1][0]) / 2, U[k][2]]); x = Phi @ x + Gam @ u; out.append(C @ x + Dm @ u)
    return np.array(out)
def fitG(A, B, C):
    """3-state model y_{k+1} = F y_k + G u_k from the core's simulated responses"""
    Phi, Gam = disc(A, B); Xr, Yr = [], []
    for run in range(200):
        x = np.zeros(A.shape[0]); u = np.zeros(2); y = C @ x
        for k in range(30):
            if k % 6 == 0: u[0] = (rng.random() - 0.5) * 2 / 57.3
            if k % 15 == 0: u[1] = (rng.random() - 0.5) * 0.1
            x = Phi @ x + Gam @ u; y2 = C @ x; Xr.append(np.r_[y, u]); Yr.append(y2); y = y2
    Xr, Yr = np.array(Xr), np.array(Yr); Th = np.linalg.lstsq(Xr, Yr, rcond=None)[0].T
    return Th[:, :3], Th[:, 3:]
def sim_G(F, G, U):
    y = np.zeros(3); out = [y.copy()]
    for k in range(30): y = F @ y + G @ np.array([U[k][0], U[k][2]]); out.append(y.copy())
    return np.array(out)
names = ['beta', 'r', 'v']; V = ['full', 'W', 'WB', 'G']; fits = {v: {e: {j: [] for j in range(3)} for e in ('steer', 'thr')} for v in V}; eigs = []
for p in D['pts']:
    if 'A' not in p or p['sd'][0] * 57.3 >= 0.5: continue
    A, B, C = np.array(p['A']), np.array(p['B']), np.array(p['C'])
    ev, W = np.linalg.eig(A); eigs.append((p['R'], p['b'], sorted(ev, key=lambda e: abs(e))))
    rW = resid(A, B, C, [3, 4, 5, 6]); rWB = resid(A, B, C, [3, 4, 5, 6, 7]); FG = fitG(A, B, C)
    for e in ('steer', 'thr'):
        Y = np.array(p[e]); dY = Y[:, :3]; U = [[Y[k][5], Y[k][6], 0.05 if e == 'thr' else 0.0] for k in range(31)]
        ys = {'full': sim_ct(A, B, C, np.zeros((3, 2)), U), 'W': sim_ct(*rW, U), 'WB': sim_ct(*rWB, U), 'G': sim_G(*FG, U)}
        for v in V:
            for j in range(3):
                nE = np.linalg.norm(dY[:, j]); fits[v][e][j].append(1 - np.linalg.norm(ys[v][:, j] - dY[:, j]) / nE)
print(f"{D['drive']}: {len(eigs)} steady points | the core's eigenvalues (1/s), smallest |λ| first, at a few points:")
for R, b, ev in eigs[:6]: print(f"  {R}/{b}: " + ' '.join(f"{e.real:+.1f}{e.imag:+.1f}j" if abs(e.imag) > 1e-6 else f"{e.real:+.2f}" for e in ev))
print("median fit (1 = exact):")
for v in V:
    print(f"  {v:4} " + ' | '.join(f"{e} " + ' '.join(f"{names[j]} {np.median(fits[v][e][j]):+.2f}" for j in range(3)) for e in ('steer', 'thr')))
