const {loadEngine}=require('./load');const f=(x,d=1)=>(+x).toFixed(d);const [tr,surface,drive]=process.argv.slice(2);
const E=loadEngine();Object.assign(E.cfg,{surface,drive,abs:false,auto:true,steer:'direct',uniform:false,profile:true,driven:true,hazards:'off'});E.VERT.amp=1;E.applyDriveSetup(drive);E.selectTrack(tr);
const S=E.S,T=E.TRACK,L=T.pts[T.N-1].s;let t=0,laps=0,prevS=T.pts[S.idx].s,tLap=null,off=0;
while(t<300&&laps<2){E.ghostAI();E.step(E.DT);t+=E.DT;const s=T.pts[S.idx].s;if(s<prevS-L/2){laps++;if(laps===1)tLap=t;if(laps===2)tLap=t-tLap;}prevS=s;if(laps>=1&&!S.onTrack)off+=E.DT;}
console.log((process.env.ENGINE||'current').padEnd(18),tr,surface,drive,laps>=2?f(tLap)+' s, off '+f(off)+' s':'did not finish');
