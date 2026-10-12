// The measured-envelope driver: mode true (envelope + speed profile + recovery, old layers) or 'catch' (the autopilot Y:
// envelope profile + the tutor's catch). node env.js track surface drive pace [catch|old] [1 = yaw kicks ±0.6 rad/s every 7 s]
const {loadEngine}=require('./load');const f=(x,d=1)=>(+x).toFixed(d);const [tr,surface,drive,pace,mode,kickA,driftA]=process.argv.slice(2);const kick=kickA==='1';let nk=0;
const E=loadEngine();Object.assign(E.cfg,{surface,drive,abs:false,auto:true,steer:'direct',uniform:false,profile:true,driven:true,hazards:'off'});E.VERT.amp=1;E.applyDriveSetup(drive);E.selectTrack(tr);
E.AP.on=true;E.AP.pace=+pace;E.AP.drift=+(driftA||0);E.apEnsure();while(E.AP.job)E.apRun(1e9);E.apEnsure();
let lineMsg='ideal line';if(process.env.NOLINE){E.AP.lineJob=null;E.AP.line=null;}else{const t0=Date.now();while(E.AP.lineJob)E.apLineRun(1e9);E.apEnsure();if(!process.env.NOPRE){while(E.AP.preJob)E.apPreRun(1e9);E.apEnsure();}const I=E.AP.lineInfo;
  if(I)lineMsg='racing line (planned '+f(I.T0,2)+' → '+f(I.T,2)+' s, '+I.evals+' evaluations, '+f((Date.now()-t0)/1000,1)+' s)';}
const S=E.S,T=E.TRACK,L=T.pts[T.N-1].s,mu=E.SURFX().mu;let t=0,laps=0,prevS=T.pts[S.idx].s,tLap=null,off=0,sp=0,use=0,n=0,bS=0,bR=0,bN=0;
while(t<300&&laps<2){if(Math.round(t/E.DT)%360===0)E.apEnsure();const c=E.ghostCommand(E.IN.brk,E.DT,mode==='catch'?'catch':true);if(kick&&laps>=1&&Math.round(t/E.DT)%(360*7)===0){S.r+=(nk++%2?0.6:-0.6);}E.IN.steer=c.steer;E.IN.thr=c.thr;E.IN.brk=c.brk;E.IN.hb=c.hb||0;E.step(E.DT);t+=E.DT;const s=T.pts[S.idx].s;if(s<prevS-L/2){laps++;if(laps===1)tLap=t;if(laps===2)tLap=t-tLap;}prevS=s;
  if(laps>=1){if(!S.onTrack)off+=E.DT;if(Math.abs(Math.atan2(S.vy,Math.abs(S.vx)))>1.2)sp+=E.DT;use+=Math.hypot(S.ax,S.ay)/(mu*9.81);n++;if(Math.abs(E.AP.brNow||0)>5/57.3){bS+=Math.abs(Math.atan2(S.vy,Math.abs(S.vx)));bR+=Math.abs(E.AP.brNow);bN++;}}}
console.log((mode==='catch'?'race':'envelope')+(kick?'+'+nk+' kicks':''),tr,surface,drive,'pace',pace,laps>=2?f(tLap)+' s, off '+f(off)+' s, spinning '+f(sp)+' s, mean tyre use '+f(use/n,2)+(bN?', slip '+f(bS/bN*57.3)+'° of '+f(bR/bN*57.3)+'°':'')+' | '+lineMsg:'did not finish');
