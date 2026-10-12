#!/bin/bash
# Acceptance suite for changes to the autopilot (race plan, slides, line steering).
# Bar: every case on the road ("off 0.0 s"). Runs two lanes in parallel, ~10 min on two cores.
# Usage: cd test && python3 extract.py && bash acceptance.sh [out-dir]
#   ENGINE=other-engine.js bash acceptance.sh   compares another extracted build.
# Each line: track surface drive setup, limit-range setting (drift), laps, held slip angle against the
# planned one, handbrake use, time off the road, time the catch acted. drstat.js drives as the browser
# does (inputs reset every frame, the plan computed first).
cd "$(dirname "$0")"
OUT=${1:-./acceptance-out}; mkdir -p "$OUT"; rm -f "$OUT"/case.*
f(){ grep -v "^   " | sed 's/| driftMap.*//'; }
r(){ k=$1; shift; env "$@" 2>&1 | f > "$OUT/case.$k"; }
(
r a SEED=19029 timeout 900 node drstat.js random 1 1          # random stage 19029, slides 100 %
r b SEED=19029 timeout 900 node drstat.js random 0.7 1        # random stage 19029, slides 70 %
r c DRIVE=rwd timeout 900 node drstat.js rundkurs 1 2         # circuit gravel RWD, slides 100 %
r d timeout 900 node drstat.js rundkurs 1 2                   # circuit gravel AWD, slides 100 %
r e timeout 900 node drstat.js rundkurs 0 2                   # circuit gravel AWD, grip
) &
(
r f SEED=9968 timeout 900 node drstat.js random 1 1           # random stage 9968, slides 100 %
r g SEED=9968 timeout 900 node drstat.js random 0 1           # random stage 9968, grip
r h DRIVE=fwd timeout 900 node drstat.js etappe 0 1           # stage gravel FWD, grip
r i DRIVE=fwd timeout 900 node drstat.js rundkurs 1 2         # circuit gravel FWD, slides 100 %
r j DRIVE=rwd timeout 900 node drstat.js etappe 1 1           # stage gravel RWD, slides 100 %
r k SEED=4711 timeout 900 node drstat.js random 1 1           # another random stage, slides 100 %
) &
wait
cat "$OUT"/case.? | tee "$OUT/summary.txt"
