#!/usr/bin/env python3
"""Robustness of the drift controller's gains (decision 10 Oct), from the models dc/robust.js wrote (OUT=...json).

For every point: the augmented system of each repeated identification (vehicle 6 states, the driver's hand target p,
the delay queue, the integrals of slip and radial error – as dcGain builds it), the LQR gain of each, and the largest
closed-loop eigenvalue modulus of every gain on every model of the point (per 1/60 s frame; < 1 stable). A design
counts as robust at a point when all gain/model pairs are stable.

python3 dc/robust.py models.json [Q=1,0.05,0.2,1,1,0] [RW=10,10] [QI=0.1,0.5]
"""
import json, sys
import numpy as np
from scipy.linalg import solve_discrete_are

def args():
    kv = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    return {k: [float(x) for x in v.split(',')] for k, v in kv.items()}

def aug(M, ch):
    n, nd, a = 6, ch['nd'], ch['a']
    N = n + 1 + nd + 2
    A = np.zeros((N, N)); B = np.zeros((N, 2))
    A[:n, :n] = np.array(M['Ad']); A[:n, n] = np.array(M['Bd'])[:, 0]; B[:n, 1] = np.array(M['Bd'])[:, 1]
    A[n, n] = a
    if nd > 0: A[n, n + nd] = 1 - a
    else: B[n, 0] = 1 - a
    for j in range(nd):
        if j == 0: B[n + 1, 0] = 1
        else: A[n + 1 + j, n + j] = 1
    A[N - 2, 0] = 1 / 60; A[N - 2, N - 2] = 1; A[N - 1, 3] = 1 / 60; A[N - 1, N - 1] = 1
    return A, B

def kin(M, R):
    """the path rows from geometry (decision 10 Oct): e' = -v sin h, h' = r + beta' - v cos h/(R+e), linearised at x0;
    beta' from the model's own slip row"""
    M = dict(M); Ad = [list(r) for r in M['Ad']]; Bd = [list(r) for r in M['Bd']]; cd = list(M.get('cd', [0] * 6)); x0 = M['x0']; dt = 1 / 60
    b0, r0, v0, e0, h0, d0 = x0; rho0 = R + e0
    Je = [0, 0, -np.sin(h0), 0, -v0 * np.cos(h0), 0]; ge = -v0 * np.sin(h0)
    Ad[3] = [(1 if j == 3 else 0) + dt * Je[j] for j in range(6)]; Bd[3] = [0, 0]; cd[3] = dt * (ge - sum(Je[j] * x0[j] for j in range(6)))
    Jh = [0, 1, -np.cos(h0) / rho0, v0 * np.cos(h0) / rho0 ** 2, v0 * np.sin(h0) / rho0, 0]; gh = r0 - v0 * np.cos(h0) / rho0
    Ad[4] = [(1 if j == 4 else 0) + M['Ad'][0][j] - (1 if j == 0 else 0) + dt * Jh[j] for j in range(6)]; Bd[4] = list(M['Bd'][0])
    cd[4] = (M.get('cd', [0] * 6)[0]) + dt * (gh - sum(Jh[j] * x0[j] for j in range(6)))
    M['Ad'], M['Bd'], M['cd'] = Ad, Bd, cd; return M

def gain(A, B, Q, QI, RW, SC, SU):
    N = A.shape[0]
    qd = np.zeros(N)
    for i in range(6): qd[i] = Q[i] / SC[i] ** 2
    qd[N - 2] = QI[0] * 57.3 * 57.3; qd[N - 1] = QI[1] / 0.25
    Rm = np.diag([RW[i] / SU[i] ** 2 for i in range(2)])
    P = solve_discrete_are(A, B, np.diag(qd), Rm)
    return np.linalg.solve(Rm + B.T @ P @ B, B.T @ P @ A)

def rho(A): return max(abs(np.linalg.eigvals(A)))

def main():
    D = json.load(open(sys.argv[1])); o = args()
    Q, QI, RW = o.get('Q', D['Q']), o.get('QI', D['QI']), o.get('RW', D['RW'])
    ch = D['ch']; worst_all = 0
    for p in D['pts']:
        Ms = [m for m in p['runs']] + ([p['tab']] if p.get('tab') and o.get('TAB', [1])[0] else [])
        if len(Ms) < 2: print(f"R{p['R']}/{p['b']}: {len(Ms)} model(s), skipped"); continue
        if o.get('KIN', [0])[0]: Ms = [kin(M, p['R']) for M in Ms]
        sys_ = [aug(M, ch) for M in Ms]
        Ks = [gain(A, B, Q, QI, RW, D['SC'], D['SU']) for (A, B), M in zip(sys_, Ms) if M.get('cond', 'n') == 'n']   # the gains: from the identification's own condition (as the game designs them)
        tab = np.zeros((len(Ks), len(sys_)))
        keep = [k for k in range(sys_[0][0].shape[0]) if not (k == sys_[0][0].shape[0] - 2 and QI[0] == 0 or k == sys_[0][0].shape[0] - 1 and QI[1] == 0)]
        for i, K in enumerate(Ks):
            for j, (A, B) in enumerate(sys_): Acl = A - B @ K; tab[i, j] = rho(Acl[np.ix_(keep, keep)])   # an integral not weighted: its state is dropped (its unit eigenvalue is not the loop's)
        ol = [rho(np.array(M['Ad'])) for M in Ms]
        worst = tab.max(); worst_all = max(worst_all, worst)
        conds = sorted(set(M.get('cond', 'n') for M in Ms)); byc = {c: max(tab[i, j] for i in range(len(Ks)) for j, M in enumerate(Ms) if M.get('cond', 'n') == c) for c in conds}
        print(f"R{p['R']}/{p['b']}: worst {worst:.4f}  " + ' '.join(f'{c}:{v:.4f}' for c, v in byc.items()) + f"  open loop {min(ol):.4f}–{max(ol):.4f}"
              + ('  (last row/col: the table model)' if p.get('tab') else ''))
        if 'V' in o:
            for i in range(len(Ks)): print('   ', ' '.join(f'{v:.4f}' for v in tab[i]))
            A, B = sys_[0]; w, V = np.linalg.eig(A - B @ Ks[0]); k = np.argmax(abs(w)); v = V[:, k]; v = v / v[np.argmax(abs(v))]
            names = ['b', 'r', 'v', 'e', 'h', 'd', 'p'] + [f'q{j}' for j in range(ch['nd'])] + ['zb', 'ze']
            print('    slowest mode', f'{w[k]:.4f}', ' '.join(f'{nm}:{abs(c):.2f}' for nm, c in zip(names, v) if abs(c) > 0.05))
    print(f'WORST {worst_all:.4f}')

main()
