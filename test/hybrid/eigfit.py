#!/usr/bin/env python3
"""Do the fitted 60 Hz controller models keep the core's slow dynamics? (decision 11 Oct: the game's fitted models had an
unstable pair +2 ± 1.2j /s where the core's slowest modes are near −0.0 ± 0.3j – the regulator then needs a minimum
gain whatever the weights.) Per run length of the fit: the fitted model's eigenvalues (β r v δ) against the core's
slow ones, and the fit to the engine's steps.  python3 hybrid/eigfit.py ENG.json"""
import json, sys
import numpy as np
from scipy.linalg import expm
D = json.load(open(sys.argv[1])); dt = 1 / 60
def disc(A, B):
    n = A.shape[0]; m = B.shape[1]; M = np.zeros((n + m, n + m)); M[:n, :n] = A; M[:n, n:] = B; E = expm(M * dt); return E[:n, :n], E[:n, n:]
def fit(A, B, C, T, runs, hold_d=6, hold_t=15, amp=1):
    rng = np.random.default_rng(1); Phi, Gam = disc(A, B); Y0, U, Y1 = [], [], []
    for run in range(runs):
        x = np.zeros(A.shape[0]); u = np.zeros(2)
        for k in range(T):
            if k % hold_d == 0: u[0] = (rng.random() - 0.5) * 2 * amp / 57.3
            if k % hold_t == 0: u[1] = (rng.random() - 0.5) * 0.1
            y = C @ x; x = Phi @ x + Gam @ u; Y0.append(y); U.append(u.copy()); Y1.append(C @ x)
    Y0, U, Y1 = map(np.array, (Y0, U, Y1)); F = np.zeros((4, 4)); G = np.zeros((4, 2))
    Xv = np.c_[Y0, U[:, 1]]
    for j in range(3): th = np.linalg.lstsq(Xv, Y1[:, j], rcond=None)[0]; F[j] = th[:4]; G[j, 1] = th[4]
    th = np.linalg.lstsq(np.c_[Y0[:, 3], U[:, 0]], Y1[:, 3], rcond=None)[0]; F[3, 3] = th[0]; G[3, 0] = th[1]
    return F, G
cont = lambda ev: np.log(ev.astype(complex)) / dt
fmt = lambda ev: ' '.join(f"{e.real:+.2f}{e.imag:+.2f}j" for e in sorted(ev, key=lambda e: -e.real)[:3])
for p in D['pts'][:40:5]:
    if 'As' not in p: continue
    A, B, C = np.array(p['As']), np.array(p['Bs']), np.array(p['Cs'])
    evc = np.linalg.eigvals(A); slow = sorted([e for e in evc if abs(e) < 8], key=lambda e: -e.real)
    print(f"{p['R']}/{p['b']} core slow: {fmt(np.array(slow))}")
    for T, runs in ((30, 200), (120, 60), (360, 20), (360, 20)):
        F, G = fit(A, B, C, T, runs, *( (30, 60, 1) if (T, runs) == (360, 20) and _ else (6, 15, 1) ) ) if False else fit(A, B, C, T, runs)
        print(f"   runs {T/60:.1f} s: fitted {fmt(cont(np.linalg.eigvals(F)))}")
