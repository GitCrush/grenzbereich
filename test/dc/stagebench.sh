#!/bin/bash
# The drift controller on stages (decision 11 Oct: stage corners are short and narrow – few, so one stage says little):
# random stages 19029, 9968, 4711, 777 and the stage track, two lanes; corners, given up, off, within ±4°.
# dc/stagebench.sh TAG 'SETA-json'   (LS: cache directory; DRIVE passes through)
TAG=${1:-run}; SA=${2:-{\}}; LS=${LS:-/tmp/dcstage}; mkdir -p $LS; D=$(dirname "$0")/..
run(){ local l=$1; shift; env V=1 NL=1 TAG=$TAG SETA="$SA" LSFILE=$LS/ls_$l.json "$@" node $D/dc-entry.js; }
( run a SEED=19029; run a SEED=777 ) > $LS/$TAG.a 2>&1 &
( run b SEED=4711; run b SEED=9968; run b TRACK=etappe ) > $LS/$TAG.b 2>&1 &
wait; cat $LS/$TAG.a $LS/$TAG.b | grep -v "^   corner" | awk -v t=$TAG '{print} /corners/{for(i=1;i<=NF;i++){if($i=="corners")n+=$(i+1);if($i=="up")g+=$(i+1);if($i=="off")o+=$(i+1)}} END{printf "%s STAGES corners %d given up %d off %.1f s\n",t,n,g,o}'
