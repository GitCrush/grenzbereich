#!/bin/bash
# dc/bench.sh over three slide tables measured with other excitation sequences (AP.dcSeedOff 0, 1000, 2000): the
# outcome of a corner depended on the table measured (decision 10 Oct) – compare variants over all three.
# dc/bench3.sh TAG 'SETA-json' [laps]   (LS: cache directory, one per table)
TAG=${1:-run}; SA=${2:-{\}}; NL=${3:-2}; LS=${LS:-/tmp/dcbench3}; D=$(dirname "$0")
for so in 0 1000 2000; do
  sa=$(python3 -c "import json,sys;d=json.loads(sys.argv[1]);d['dcSeedOff']=$so;print(json.dumps(d))" "$SA")
  LS=$LS/t$so $D/bench.sh ${TAG}_t$so "$sa" $NL | tail -1
done | awk -v t=$TAG '{print} {for(i=1;i<=NF;i++){if($i=="corners")n+=$(i+1);if($i=="up")g+=$(i+1);if($i=="off")o+=$(i+1)}} END{printf "%s ALL TABLES corners %d given up %d off %.1f s\n",t,n,g,o}'
