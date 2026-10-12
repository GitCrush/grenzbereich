#!/usr/bin/env python3
"""Summary of the hybrid's go/no-go test against the engine (decision 11 Oct): per file of hybrid/engstep.js, the
physical core's steady slide against the engine's clean hold (only holds that were steady: beta sd < SD deg and delta sd < 2 SD; the mean
of a swinging hold is no steady slide) and the step-response fits of hybrid/eng.py.
python3 hybrid/summary.py ENG.json [ENG.json …] [SD=0.5]"""
import json, sys, subprocess
import numpy as np
SD = float(next((a[3:] for a in sys.argv[1:] if a.startswith('SD=')), 0.5))
for f in [a for a in sys.argv[1:] if not a.startswith('SD=')]:
    D = json.load(open(f)); e = {'v': [], 'd': [], 'thr': []}; n = len(D['pts']); st = 0; nok = 0
    for p in D['pts']:
        if p['sd'][0] * 57.3 >= SD or p['sd'][5] * 57.3 >= 2 * SD: continue   # steady: slip and steering (the 1.3 Hz steering swing leaves the slip steady)
        st += 1
        if not p['eq']['ok']: continue
        nok += 1; m = p['mean']; q = p['eq']
        e['v'].append(100 * (q['v'] - m[2]) / m[2]); e['d'].append((q['delta'] - m[5]) * 57.3); e['thr'].append(q['thr'] - m[7])
    s = lambda a: f"median|.| {np.median(np.abs(a)):.2f} rms {np.sqrt(np.mean(np.square(a))):.2f} max|.| {np.max(np.abs(a)):.2f}" if a else '–'
    print(f"{f.split('/')[-1]}: {D['drive']} {n} holds, {st} steady (β sd < {SD}°, δ sd < {2*SD}°), model converged at {nok}")
    print(f"   speed % {s(e['v'])} | δ ° {s(e['d'])} | throttle {s(e['thr'])}")
