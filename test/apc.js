// Phantom driver vs phantom + catch: two laps (second timed), optionally with yaw kicks of ±0.6 rad/s every 7 s on the timed lap.
const {loadEngine}=require('./load');const f=(x,d=1)=>(+x).toFixed(d);const [tr,surface,drive,mode,kick,drift]=process.argv.slice(2);
const E=loadEngine();Object.assign(E.cfg,{surface,drive,abs:false,auto:true,steer:'direct',uniform:false,profile:true,driven:true,hazards:'off'});E.VERT.amp=1;E.applyDriveSetup(drive);E.selectTrack(tr);E.AP.drift=+(drift||0);
const S=E.S,T=E.TRACK,L=T.pts[T.N-1].s;let bR=0,t=0,laps=0,prevS=T.pts[S.idx].s,tLap=null,off=0,k=0,sp=0,bS=0,bN=0;
while(t<300&&laps<2){const c=E.ghostCommand(E.IN.brk,E.DT,mode==='catch'?'catch':false);E.IN.steer=c.steer;E.IN.thr=c.thr;E.IN.brk=c.brk;E.IN.hb=c.hb||0;
  if(kick==='1'&&laps>=1&&Math.round(t/E.DT)%(360*7)===0){S.r+=(k++%2?0.6:-0.6);}
  E.step(E.DT);t+=E.DT;const s=T.pts[S.idx].s;if(s<prevS-L/2){laps++;if(laps===1)tLap=t;if(laps===2)tLap=t-tLap;}prevS=s;if(laps>=1){if(!S.onTrack)off+=E.DT;if(Math.abs(Math.atan2(S.vy,Math.abs(S.vx)))>1.2)sp+=E.DT;if(Math.abs(E.AP.brNow||0)>5/57.3){bS+=Math.abs(Math.atan2(S.vy,Math.abs(S.vx)));bR=(bR||0)+Math.abs(E.AP.brNow);bN++;}}}
console.log((mode+(kick==='1'?'+kicks':'')).padEnd(14),tr,surface,drive,laps>=2?f(tLap)+' s, off '+f(off)+' s, spinning '+f(sp)+' s, slip where drifting '+f(bS/Math.max(1,bN)*57.3)+' deg (target '+f(bR/Math.max(1,bN)*57.3)+')':'did not finish');
