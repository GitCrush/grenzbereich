// Autopilot laps: the phantom's driver on the measured envelope and speed profile, two laps, the second timed.
const {loadEngine}=require('./load');const f=(x,d=1)=>(+x).toFixed(d);
function lap(track,surface,drive,pace,seed){const E=loadEngine();Object.assign(E.cfg,{surface,drive,abs:false,auto:true,steer:'direct',uniform:false,profile:true,driven:true,hazards:'off'});E.VERT.amp=1;E.applyDriveSetup(drive);
  if(track==='random'){E.TRACKS.random=E.randomTrackDef(seed,0.6,0.6);}E.selectTrack(track);
  if(E.AP){if(pace)E.AP.pace=pace;E.apEnsure();while(E.AP.job)E.apRun(1e9);E.apEnsure();}
  const S=E.S,T=E.TRACK,L=T.pts[T.N-1].s;let t=0,laps=0,prevS=T.pts[S.idx].s,tLap=null,off=0;
  while(t<400&&laps<2){E.ghostAI();E.step(E.DT);t+=E.DT;const s=T.pts[S.idx].s;if(s<prevS-L/2){laps++;if(laps===1)tLap=t;if(laps===2)tLap=t-tLap;}prevS=s;if(laps>=1&&!S.onTrack)off+=E.DT;}
  return laps>=2?f(tLap)+' s, off '+f(off)+' s':'did not finish';}
const a=process.argv.slice(2);console.log(a.join(' ').padEnd(30),lap(a[0],a[1],a[2],a[3]?+a[3]:0,+a[4]||7));
