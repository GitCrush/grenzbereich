#!/bin/bash
# The drift controller over its corner set (decision 10 Oct: a lap of an oval is two corners and single runs flipped
# between held and lost with changes that should not matter – compare variants over all of them).
# dc/bench.sh TAG 'SETA-json' [laps]   – ovals R 40/55/83/120 m and the circuit, two at a time (two cores);
# the envelope, slide table and racing lines are kept per lane in $LS (delete when the identification changes).
TAG=${1:-run}; SA=${2:-{\}}; NL=${3:-3}; LS=${LS:-/tmp/dcbench}; mkdir -p $LS; D=$(dirname "$0")/..
run(){ local l=$1; shift; env V=1 NL=$NL TAG=$TAG SETA="$SA" LSFILE=$LS/ls_$l.json "$@" node $D/dc-entry.js; }
( run a OVALS="40,150 83,250" ; run a TRACK=rundkurs ) > $LS/$TAG.a 2>&1 &
( run b OVALS="55,200 120,250" ) > $LS/$TAG.b 2>&1 &
wait; cat $LS/$TAG.a $LS/$TAG.b | grep -v "^   corner\| ALL" | awk -v t=$TAG '{print} /corners/{for(i=1;i<=NF;i++){if($i=="corners")n+=$(i+1);if($i=="up")g+=$(i+1);if($i=="off")o+=$(i+1)}} END{printf "%s TOTAL corners %d given up %d off %.1f s\n",t,n,g,o}'
