#!/bin/bash
# The corner check as a benchmark (12 Oct): the controller's candidate corners on five tracks for RWD and AWD, each driven
# by the corner check (the car started before the corner, the controller takes the slide) – counted: kept, given up, off
# the road. Single corners flip with any change to the controller; the count over ~40 corners judges a variant.
# Usage: cd test && SETA='{"dcSat":true}' bash dc/chkbench.sh OUT   (the looser corner criteria dcRR 2.5, dcMinT 2 are set;
#   SETA adds to them). Two lanes, ~6 min.
cd "$(dirname "$0")/.."
OUT=${1:-./chkbench-out}; mkdir -p "$OUT"; X=${SETA:-'{}'}
S=$(python3 -c "import json,sys;a={'dcRR':2.5,'dcMinT':2};a.update(json.loads(sys.argv[1]));print(json.dumps(a))" "$X")
jobs=("TRACK=etappe" "TRACK=rundkurs" "SEED=4711" "SEED=9968" "SEED=19029")
run(){ local d=$1 t=$2 o="$OUT/$1.${2#*=}"; env DRIVE=$d $t SETA="$S" timeout 600 node hybrid/corners.js > "$o" 2>&1; }
( for t in "${jobs[@]}"; do run rwd "$t"; done ) & ( for t in "${jobs[@]}"; do run awd "$t"; done ) & wait
for d in rwd awd; do k=$(cat "$OUT"/$d.* | grep -c ': kept'); n=$(cat "$OUT"/$d.* | grep -c '^corner'); g=$(cat "$OUT"/$d.* | grep -c 'given up true'); f=$(cat "$OUT"/$d.* | grep -c 'off true'); echo "$d: $k of $n kept, $g given up, $f off  ($S)"; done | tee "$OUT/summary.txt"
