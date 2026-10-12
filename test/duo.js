// Autopilot and phantom on the road together, as in the browser (G and Y): both from a standing start, the
// browser's frame loop (per-frame input reset, real-time slices for the plan). Lap times of both, excursions of both.
// env: SEED, DRIFT (limit range of the autopilot), GHOSTPACE, YAT (s before Y; the phantom starts at once)
const {loadEngine}=require('./load');const f=(x,d=1)=>(+x).toFixed(d);const T_END=+(process.argv[2]||200);
const E=loadEngine({realTime:true});if(process.env.DRIFT)E.AP.drift=+process.env.DRIFT;if(process.env.GHOSTPACE)E.GAI.aLat=+process.env.GHOSTPACE;
if(process.env.SEED){E.TRACKS.random=E.randomTrackDef(+process.env.SEED);E.selectTrack('random');}else E.selectTrack(process.argv[3]||'rundkurs');
E.spawnGhost();const S=E.S,T=E.TRACK,L=T.pts[T.N-1].s,YAT=+(process.env.YAT||3);
let t=0,frame=0;const car={prev:T.pts[S.idx].s,laps:[],tl:0,off:0},gh={prev:T.pts[E.ghost.S.idx].s,laps:[],tl:0,off:0};
while(t<T_END){E.IN.steer=0;E.IN.thr=0;E.IN.brk=0;E.IN.hb=0;   // readInput
  if(!E.AP.on&&t>=YAT){E.AP.on=true;E.AP.lineOn=false;E.AP.pedBrk=0;}
  if(frame%30===0||!E.AP.prof)E.apEnsure();{const ms=(!E.AP.on&&Math.hypot(S.vx,S.vy)<1)?10:5;if(E.AP.job)E.apRun(ms);else if(E.AP.lineJob)E.apLineRun(ms);}
  for(let k=0;k<6;k++){if(E.AP.on){const c=E.ghostCommand(E.AP.pedBrk||0,E.DT,'catch');E.AP.pedBrk=c.brk;E.IN.steer=c.steer;E.IN.thr=c.thr;E.IN.brk=c.brk;E.IN.hb=c.hb||0;}
    E.step(E.DT);E.stepGhost(E.DT);t+=E.DT;
    for(const [o,st] of [[car,S],[gh,E.ghost.S]]){const s=T.pts[st.idx].s;if(t>=(o===car?YAT:0))o.tl+=E.DT;if(s<o.prev-L/2){o.laps.push(f(o.tl));o.tl=0;}o.prev=s;if(!st.onTrack)o.off+=E.DT;}}
  frame++;}
console.log('seed',process.env.SEED||'-','drift',E.AP.drift,'ghost pace',E.GAI.aLat,'| AUTOPILOT laps',car.laps.join(', '),'off',f(car.off),'s | PHANTOM laps',gh.laps.join(', '),'off',f(gh.off),'s');
