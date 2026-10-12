#!/bin/bash
# The acceptance suite's slide cases, each driven several times from a start moved a few centimetres to the side
# (drstat.js JIT=k: k × 5 cm), for two settings side by side. Why (11 Oct): with the edge distance measured, the default
# passed 11 of 11 with two cases 0.0 m from the edge and three within 0.3 m – any change upstream (the controller on, a
# wider line margin) moved one of them over the edge and another back, so one run per case cannot tell a better setting
# from a lucky one. Counted here: per case and setting, runs off the road and the closest approach to the edge.
# Usage: cd test && python3 extract.py && A='{}' B='{"dcUse":true}' JITS="0 1 2 3" bash acc-robust.sh [out-dir]
#   CASES="a b c d f i j k" (default; the grip cases e g h do not use the slide logic), two lanes in parallel.
cd "$(dirname "$0")"
OUT=${1:-./robust-out}; mkdir -p "$OUT"
A=${A:-'{}'}; B=${B:-'{"dcUse":true}'}; JITS=${JITS:-"0 1 2 3"}; CASES=${CASES:-"a b c d f i j k"}
f(){ grep -v "^   " | sed 's/| driftMap.*//'; }
run(){ # case setting-name setting jit
  local k=$1 nm=$2 st=$3 j=$4 o="$OUT/$2.j$4.$1"; [ -s "$o" ] && return; mkdir "$o.lock" 2>/dev/null || return   # a lane claims a run
  case $k in
    a) env SETA="$st" JIT=$j SEED=19029 timeout 900 node drstat.js random 1 1 ;;
    b) env SETA="$st" JIT=$j SEED=19029 timeout 900 node drstat.js random 0.7 1 ;;
    c) env SETA="$st" JIT=$j DRIVE=rwd timeout 900 node drstat.js rundkurs 1 2 ;;
    d) env SETA="$st" JIT=$j timeout 900 node drstat.js rundkurs 1 2 ;;
    e) env SETA="$st" JIT=$j timeout 900 node drstat.js rundkurs 0 2 ;;
    f) env SETA="$st" JIT=$j SEED=9968 timeout 900 node drstat.js random 1 1 ;;
    g) env SETA="$st" JIT=$j SEED=9968 timeout 900 node drstat.js random 0 1 ;;
    h) env SETA="$st" JIT=$j DRIVE=fwd timeout 900 node drstat.js etappe 0 1 ;;
    i) env SETA="$st" JIT=$j DRIVE=fwd timeout 900 node drstat.js rundkurs 1 2 ;;
    j) env SETA="$st" JIT=$j DRIVE=rwd timeout 900 node drstat.js etappe 1 1 ;;
    k) env SETA="$st" JIT=$j SEED=4711 timeout 900 node drstat.js random 1 1 ;;
  esac 2>&1 | f > "$o"; rmdir "$o.lock"
}
jobs=(); for j in $JITS; do for k in $CASES; do jobs+=("$k A $j" "$k B $j"); done; done
# each lane takes the next run no lane has claimed (the controller's runs take longer: they compute its table first)
rm -rf "$OUT"/*.lock
lane(){ for jb in "${jobs[@]}"; do set -- $jb; if [ $2 = A ]; then run $1 A "$A" $3; else run $1 B "$B" $3; fi; done; }
lane & sleep 1; lane & wait
# summary: per case and setting – runs off, total time off, closest approach to the edge (worst run)
python3 - "$OUT" "$A" "$B" <<'EOF'
import sys,glob,re,os
out=sys.argv[1];R={}
for p in glob.glob(out+'/[AB].j*.?'):
  nm,j,k=os.path.basename(p).split('.');t=open(p).read()
  m=re.search(r'off ([\d.]+) s, edge (-?[\d.]+) m at s (\d+)',t)
  if m:R.setdefault((k,nm),[]).append((float(m.group(1)),float(m.group(2)),int(m.group(3)),j))
print('A =',sys.argv[2],' B =',sys.argv[3])
print('case | A: off runs, worst edge (m @ s) | B: off runs, worst edge (m @ s)')
for k in sorted({k for k,_ in R}):
  row=[k]
  for nm in 'AB':
    r=R.get((k,nm),[])
    if not r:row.append('–');continue
    w=min(r,key=lambda x:x[1]);row.append(f"{sum(1 for x in r if x[0]>0)}/{len(r)} off ({sum(x[0] for x in r):.1f} s), {w[1]:+.1f} m @ {w[2]} ({w[3]})")
  print(' | '.join(row))
EOF
