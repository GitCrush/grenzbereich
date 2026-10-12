// The game's slide model (phPoint, index.html) against the go/no-go test's model (hybrid/phys.js, ruts off) – the
// port must give the same steady slides (decision 11 Oct): the test model's residuals at the game's solution (uniform
// band under all wheels, as phRoad) and the timing per grid point.
const {loadEngine}=require('../load');const {makeModel}=require('./phys');const f=(x,d=2)=>(+x).toFixed(d);
for(const drive of ['awd','rwd']){const E=loadEngine();E.cfg.drive=drive;E.applyDriveSetup(drive);E.selectTrack('rundkurs');const P=E.phSetup(),T=makeModel(E,{ruts:false});
  T.equilibrium(55,55,-0.3,4,[20,0.05,0.5]);   // leaves the test model's plate slip at 0.2 rad/s
  let g=drive==='awd'?[23.7,0.06,0.62]:[23.1,-0.29,0.38],worst=0,n=0;const t0=Date.now();
  for(const [R,bd] of [[75,22],[55,22],[40,22],[30,22],[22,22],[100,22],[135,22],[55,16],[55,28],[100,16]]){const b=bd/57.3;
    const M=E.phPoint(P,R,b,[g[0]*Math.sqrt(R/75),g[1],g[2]]);n++;if(!M.q||M.q.fn>1e-6){console.log(drive,R,bd,'game: no slide');continue;}
    const road=[0,1,2,3].map(()=>({g:E.ROAD[E.cfg.surface].in,loose:0,hz:0,sl:0})),r=T.resid(M.q.z,R,-b,M.q.gi,road);
    const nr=Math.sqrt(r.res.reduce((a,x)=>a+x*x,0));worst=Math.max(worst,nr);
    console.log(`${drive} R${R}/${bd}: v ${f(M.q.v*3.6,1)} δ ${f(M.q.delta*57.3)} p ${f(M.q.p*57.3)} (test model p ${f(r.p*57.3)}) thr ${f(M.q.thr,3)} gear ${M.q.gi} | test model's residual ${nr.toExponential(1)}`);}
  console.log(drive,'worst residual',worst.toExponential(1),'| ms per point (band + loose, both fits):',((Date.now()-t0)/n).toFixed(0));}
