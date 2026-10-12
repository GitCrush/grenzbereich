// Autopilot with drift: lap time, slip angle in the corners, time off the road, spins.
const {loadEngine}=require('./load');const DEG=180/Math.PI;const f=(x,d=1)=>(+x).toFixed(d);
function lap(track,surface,drive,drift){const E=loadEngine();Object.assign(E.cfg,{surface,drive,abs:false,auto:true,steer:'direct',uniform:false,profile:true,driven:true,hazards:'off'});E.VERT.amp=1;E.applyDriveSetup(drive);E.selectTrack(track);
  E.AP.drift=drift;E.apEnsure();while(E.AP.job)E.apRun(1e9);E.apEnsure();
  const S=E.S,T=E.TRACK,L=T.pts[T.N-1].s;let t=0,laps=0,prevS=T.pts[S.idx].s,tLap=null,off=0,spin=0,bSum=0,bN=0,bMax=0,bRefSum=0;
  while(t<600&&laps<4){if(Math.round(t/E.DT)%360===0)E.apEnsure();E.ghostAI();E.step(E.DT);t+=E.DT;const s=T.pts[S.idx].s;if(s<prevS-L/2){laps++;if(laps===3)tLap=t;if(laps===4)tLap=t-tLap;}prevS=s;
    if(laps>=3){if(!S.onTrack)off+=E.DT;const b=Math.abs(Math.atan2(S.vy,Math.abs(S.vx)));const br=Math.abs(E.AP.prof.bref[S.idx]);if(br>5/57.3){bSum+=b;bRefSum+=br;bN++;}bMax=Math.max(bMax,b);if(b>1.2)spin+=E.DT;}}
  return laps>=4?f(tLap)+' s | off '+f(off)+' s | corner slip '+f(bSum/Math.max(1,bN)*DEG)+' deg (target '+f(bRefSum/Math.max(1,bN)*DEG)+') | max '+f(bMax*DEG,0)+' deg | spinning '+f(spin)+' s | learned scale '+f(E.AP.adapt,2):'did not finish, adapt '+f(E.AP.adapt,2)+' laps '+laps;}
const a=process.argv.slice(2);console.log(a.join(' ').padEnd(26),lap(a[0],a[1],a[2],+a[3]));
