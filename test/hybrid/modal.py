#!/usr/bin/env python3
"""Modal reduction of the physical core to the controller's basis (decision 11 Oct). The vehicle part (8 states, inputs
delta thr) keeps its K slowest modes exactly; the faster ones are made quasi-steady (singular perturbation in modal
coordinates); the slow modal coordinates are expressed in the measured ones (beta r v) – F = T Λ T⁻¹, with the fast
modes' instantaneous share as feed-through folded into the input columns. The steering row: delta's own lag, fitted to
the core with the steering. Against the engine's steps (p and throttle as recorded) and its eigenvalues; beside it the
one-step fit over 0.5 s runs (G6).  python3 hybrid/modal.py ENG.json"""
import json, sys
import numpy as np
from scipy.linalg import expm
D = json.load(open(sys.argv[1])); dt = 1 / 60
def disc(A, B):
    n = A.shape[0]; m = B.shape[1]; M = np.zeros((n + m, n + m)); M[:n, :n] = A; M[:n, n:] = B; E = expm(M * dt); return E[:n, :n], E[:n, n:]
def modal(A, B, C, k=3):
    lam, V = np.linalg.eig(A); Vi = np.linalg.inv(V); order = np.argsort(np.abs(lam)); S = list(order[:k])
    # keep complex pairs together
    if abs(lam[S[-1]].imag) > 1e-9:
        conj = [i for i in range(len(lam)) if i not in S and abs(lam[i] - np.conj(lam[S[-1]])) < 1e-6]
        if conj and len([i for i in S if abs(lam[i] - np.conj(lam[S[-1]])) < 1e-6]) == 0: S = S[:-1] + [S[-1]] if False else S[:k - 1] + [order[k]] if abs(lam[order[k]] - np.conj(lam[S[-1]])) < 1e-6 else S
    Fi = [i for i in range(len(lam)) if i not in S]
    Bm = Vi @ B; T = C @ V[:, S]; Dft = -(C @ V[:, Fi]) @ np.diag(1 / lam[Fi]) @ Bm[Fi]
    F = (T @ np.diag(lam[S]) @ np.linalg.inv(T)).real; G = (T @ Bm[S]).real - F @ Dft.real
    return F, G, Dft.real, lam[S]
def to_disc(F, G, Dft):
    Phi, Gw = disc(F, G); return Phi, Gw + Dft - Phi @ Dft
def delta_row(As, Bs, Cs):
    rng = np.random.default_rng(1); Phi, Gam = disc(As, Bs); X, Y = [], []
    for run in range(200):
        x = np.zeros(As.shape[0]); u = np.zeros(2)
        for k in range(30):
            if k % 6 == 0: u[0] = (rng.random() - 0.5) * 2 / 57.3
            if k % 15 == 0: u[1] = (rng.random() - 0.5) * 0.1
            d0 = (Cs @ x)[3]; x = Phi @ x + Gam @ u; X.append([d0, u[0]]); Y.append((Cs @ x)[3])
    return np.linalg.lstsq(np.array(X), np.array(Y), rcond=None)[0]
DRV = [2, 3] if D['drive'] == 'rwd' else [0, 1, 2, 3] if D['drive'] == 'awd' else [0, 1]; RW = 0.315
def C5(p, C):
    A = np.array(p['A']); n = A.shape[0]; c = np.zeros((2, n))
    for i in DRV: c[0, 3 + i] = RW / len(DRV)
    c[0] -= C[2]                     # mean driven wheel speed at the rim minus the car's speed (m/s)
    c[1, 7] = 1                      # boost
    return np.r_[C, c]
names = ['beta', 'r', 'v', 'delta']; fits = {v: {e: {j: [] for j in range(4)} for e in ('steer', 'thr')} for v in ('modal3', 'modal5')}; shown = 0
for p in D['pts']:
    if 'As' not in p or p['sd'][0] * 57.3 >= 0.5 or p['sd'][5] * 57.3 >= 1: continue
    A, B, C = np.array(p['A']), np.array(p['B']), np.array(p['C']); As, Bs, Cs = (np.array(p[k]) for k in ('As', 'Bs', 'Cs'))
    a_d, b_d = delta_row(As, Bs, Cs)
    for kk, nm in ((3, 'modal3'), (5, 'modal5')):
        Cx = C if kk == 3 else C5(p, C)
        try: F, G, Dft, ls = modal(A, B, Cx, kk)
        except np.linalg.LinAlgError: continue
        Phi, Gd = to_disc(F, G, Dft)
        if kk == 5 and shown < 12: print(f"   modal5 kept {' '.join(f'{l.real:+.2f}{l.imag:+.2f}j' for l in ls)}"); shown += 1
        if shown < 6 and kk == 3: print(f"{p['R']}/{p['b']}: kept modes {' '.join(f'{l.real:+.2f}{l.imag:+.2f}j' for l in ls)} | core all {' '.join(f'{l.real:+.1f}' for l in sorted(np.linalg.eigvals(A), key=abs)[:5])}"); shown += 1
        for e in ('steer', 'thr'):
            Y = np.array(p[e]); dY = Y[:, [0, 1, 2, 5]]; dthr = 0.05 if e == 'thr' else 0.0
            nv = Phi.shape[0]; yv = np.zeros(nv); yd = 0.0; out = [np.zeros(4)]
            for k in range(30):
                yv = Phi @ yv + Gd @ np.array([yd, dthr]); yd = a_d * yd + b_d * Y[k][6]; out.append(np.r_[yv[:3], yd])
            out = np.array(out)
            for j in range(4):
                nE = np.linalg.norm(dY[:, j])
                if nE > 1e-9: fits[nm][e][j].append(1 - np.linalg.norm(out[:, j] - dY[:, j]) / nE)
for nm in ('modal3', 'modal5'):
    print(f"  {nm} " + ' | '.join(f"{e} " + ' '.join(f"{names[j]} {np.median(fits[nm][e][j]):+.2f}" for j in range(4) if fits[nm][e][j]) for e in ('steer', 'thr')) + f"  (n {len(fits[nm]['steer'][0])})")
# --- the same five coordinates, fitted (one-step least squares on the core's simulated responses, 0.5 s runs) ---
def fit5(A, B, Cx, T=30, runs=200):
    rng = np.random.default_rng(1); Phi, Gam = disc(A, B); X, Y = [], []
    for run in range(runs):
        x = np.zeros(A.shape[0]); u = np.zeros(2)
        for k in range(T):
            if k % 6 == 0: u[0] = (rng.random() - 0.5) * 2 / 57.3
            if k % 15 == 0: u[1] = (rng.random() - 0.5) * 0.1
            y0 = Cx @ x; x = Phi @ x + Gam @ u; X.append(np.r_[y0, u]); Y.append(Cx @ x)
    W = np.linalg.lstsq(np.array(X), np.array(Y), rcond=None)[0].T; return W[:, :5], W[:, 5:]
f5 = {e: {j: [] for j in range(4)} for e in ('steer', 'thr')}; shown = 0
for p in D['pts']:
    if 'As' not in p or p['sd'][0] * 57.3 >= 0.5 or p['sd'][5] * 57.3 >= 1: continue
    A, B, C = np.array(p['A']), np.array(p['B']), np.array(p['C']); As, Bs, Cs = (np.array(p[k]) for k in ('As', 'Bs', 'Cs'))
    a_d, b_d = delta_row(As, Bs, Cs); Cx = C5(p, C); TF = int(next((a[3:] for a in sys.argv[2:] if a.startswith('TF=')), 30)); F, G = fit5(A, B, Cx, TF, 6000 // TF)
    if shown < 4: print(f"   {p['R']}/{p['b']} fit5 eig {' '.join(f'{e.real:+.2f}{e.imag:+.2f}j' for e in sorted(np.log(np.linalg.eigvals(F).astype(complex))/dt, key=lambda e: abs(e)))} | core {' '.join(f'{e.real:+.2f}{e.imag:+.2f}j' for e in sorted(np.linalg.eigvals(A), key=abs)[:5])}"); shown += 1
    for e in ('steer', 'thr'):
        Y = np.array(p[e]); dY = Y[:, [0, 1, 2, 5]]; dthr = 0.05 if e == 'thr' else 0.0; yv = np.zeros(5); yd = 0.0; out = [np.zeros(4)]
        for k in range(30): yv = F @ yv + G @ np.array([yd, dthr]); yd = a_d * yd + b_d * Y[k][6]; out.append(np.r_[yv[:3], yd])
        out = np.array(out)
        for j in range(4):
            nE = np.linalg.norm(dY[:, j])
            if nE > 1e-9: f5[e][j].append(1 - np.linalg.norm(out[:, j] - dY[:, j]) / nE)
print("  fit5   " + ' | '.join(f"{e} " + ' '.join(f"{names[j]} {np.median(f5[e][j]):+.2f}" for j in range(4) if f5[e][j]) for e in ('steer', 'thr')) + f"  (n {len(f5['steer'][0])})")
