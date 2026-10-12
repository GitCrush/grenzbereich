// The computed controller models' eigenvalues (1/s) – band, loose, blended – at a few grid points (decision 11 Oct).
const {loadEngine}=require('../load');const E=loadEngine();const drive=process.env.DRIVE||'awd';E.cfg.drive=drive;E.applyDriveSetup(drive);E.selectTrack('rundkurs');
const P=E.phSetup(),np=require('child_process');
function eig(A){const r=np.execFileSync('python3',['-c','import json,sys,numpy as np;A=np.array(json.loads(sys.argv[1]));print(" ".join("%+.2f%+.2fj"%((np.log(complex(e))*60).real,(np.log(complex(e))*60).imag) for e in sorted(np.linalg.eigvals(A),key=lambda e:-abs(e))[:6]))',JSON.stringify(A)]);return r.toString().trim();}
let g=drive==='awd'?[23.7,0.06,0.62]:[23.1,-0.29,0.38];
for(const [R,bd] of [[75,22],[55,22],[40,22]]){const b=bd/57.3;E.AP.tNoBlend=true;const M=E.phPoint(P,R,b,[g[0]*Math.sqrt(R/75),g[1],g[2]]);
  const lo=E.phRoad(P,true),ql=E.phEq(P,R,-b,M.q.gi,[M.q.v*0.92,M.q.delta,M.q.thr],lo,true),Ml=E.phCtl(P,ql,R,b);E.AP.tNoBlend=false;const Mb=E.phPoint(P,R,b,[g[0]*Math.sqrt(R/75),g[1],g[2]]);
  console.log(`${drive} ${R}/${bd} band: ${eig(M.Ad)}\n         loose: ${eig(Ml.Ad)}\n         blend: ${eig(Mb.Ad)}`);}
// the core's own eigenvalues (continuous, with the steering) at the same points
for(const [R,bd] of [[75,22],[55,22]]){const b=bd/57.3;E.AP.tNoBlend=true;const M=E.phPoint(P,R,b,[g[0]*Math.sqrt(R/75),g[1],g[2]]);const L=E.phLin(P,M.q);
  const r=np.execFileSync('python3',['-c','import json,sys,numpy as np;A=np.array(json.loads(sys.argv[1]));print(" ".join("%+.2f%+.2fj"%(e.real,e.imag) for e in sorted(np.linalg.eigvals(A),key=lambda e:abs(e))[:7]))',JSON.stringify(L.A)]);
  console.log(`${drive} ${R}/${bd} core (10 states, continuous): ${r.toString().trim()}`);}
