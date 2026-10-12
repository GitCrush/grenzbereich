#!/bin/bash
# chain3.sh DRIVE REACT NS M0 "R:BT ..." → identify (smooth road, per-frame excitation) bootstrapped from the previous model; validate smooth and rough
D=$(cd "$(dirname "$0")" && pwd); cd $D; export SURF=${SURF:-gravel}
DRV=$1; RE=$2; NS=$3; M0=$4; shift 4; export ZB=${ZB:-3} ZE=${ZE:-200}
for p in $@; do R=${p%%:*}; BT=${p##*:}; OUT=k_${SURF:-gravel}_${DRV}_${RE}_${NS}_${R}_${BT}.json
  EXD=6 AMPD=3 SMOOTH=1 DRIVE=$DRV REACT=$RE NS=$NS R=$R BT=$BT MODEL0=$M0 OUT=$OUT timeout 120 node ident3.js | grep -v "^ident" | sed 's/undefined/·/g' | cut -c1-200
  echo "$(TSW=${TSW:-1} MODEL=$OUT SMOOTH=1 timeout 100 node val3.js) ||$(TSW=${TSW:-1} MODEL=$OUT SMOOTH=0 timeout 100 node val3.js)"
  M0=$OUT
done
