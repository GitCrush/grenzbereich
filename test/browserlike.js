// The autopilot as the browser runs it: default settings (gravel, hazards light, uniform…), standing start, the frame loop
// (apEnsure every 30 frames, measurement and line search in 5 ms slices), 6 physics steps per frame at 60 fps.
// node browserlike.js [track] [seconds] [ghost]   env: SEED (random stage), WEATHER, BIOME, SURF, DRIVE, STEER, DRIFT, YAT (stand s before Y),
// READINPUT=0 (do not reset the car's inputs every frame as the browser's readInput does – the reset is the default), OLDPED=1 (the old pedal bug)   – logs off-track episodes with the phase the autopilot was in.
const {loadEngine}=require('./load');const f=(x,d=1)=>(+x).toFixed(d);const tr=process.argv[2]||'rundkurs',T_END=+(process.argv[3]||150),G=process.argv[4]==='ghost';
const E=loadEngine({realTime:true});   // real time slices: the plan takes as long to compute as in a browser (5–10 ms per frame)
if(process.env.SURF)E.cfg.surface=process.env.SURF;if(process.env.DRIVE){E.cfg.drive=process.env.DRIVE;E.applyDriveSetup(E.cfg.drive);}if(process.env.DRIFT)E.AP.drift=+process.env.DRIFT;if(process.env.STEER)E.cfg.steer=process.env.STEER;if(process.env.GHOSTPACE)E.GAI.aLat=+process.env.GHOSTPACE;if(process.env.WEATHER)E.cfg.weather=process.env.WEATHER;if(process.env.BIOME)E.cfg.biome=process.env.BIOME;if(process.env.SEED){const dd=E.randomTrackDef(+process.env.SEED,process.env.CURVES?+process.env.CURVES:undefined,process.env.ELEV?+process.env.ELEV:undefined);E.TRACKS.random=dd;E.selectTrack('random');}else E.selectTrack(tr);const YAT=+(process.env.YAT||0);if(!G&&!YAT)E.AP.on=true;const S=E.S,T=E.TRACK,L=T.pts[T.N-1].s;
const PH=[];let t=0,frame=0,ep=null,prevS=T.pts[S.idx].s,laps=0,tLap=0;const eps=[],lapsT=[];
const phase=()=>G?'phantom':!E.AP.prof?'heuristic':(E.AP.line&&E.AP.lineOn&&E.AP.profRace)?'racing line':'envelope, ideal line';
while(t<T_END){if(!G&&process.env.READINPUT!=='0'){E.IN.steer=0;E.IN.thr=0;E.IN.brk=0;E.IN.hb=0;}   /* readInput: the browser overwrites the player's IN with the keys every frame */
if(!G&&YAT&&!E.AP.on&&t>=YAT){E.AP.on=true;E.AP.lineOn=false;}if(!G){if(frame%30===0||!E.AP.prof)E.apEnsure();{const idle=Math.hypot(S.vx,S.vy)<1&&!E.AP.on,ms=idle?10:5;if(E.AP.job)E.apRun(ms);else if(E.AP.lineJob)E.apLineRun(ms);}}
  for(let k=0;k<6;k++){if(!G&&!E.AP.on){E.step(E.DT);t+=E.DT;continue;}const c=G?null:E.ghostCommand(process.env.OLDPED?E.IN.brk:(E.AP.pedBrk||0),E.DT,'catch');if(c)E.AP.pedBrk=c.brk;if(G)E.ghostAI();else{E.IN.direct=true;E.IN.steer=c.steer;E.IN.thr=c.thr;E.IN.brk=c.brk;E.IN.hb=c.hb||0;}E.step(E.DT);t+=E.DT;tLap+=E.DT;
    const s=T.pts[S.idx].s;if(s<prevS-L/2){lapsT.push(f(tLap)+' s ('+phase()+')');tLap=0;laps++;}prevS=s;
    if(!S.onTrack){if(!ep)ep={t0:t,s0:s,ph:phase(),v:Math.hypot(S.vx,S.vy)*3.6,b:Math.atan2(S.vy,Math.abs(S.vx))*57.3,hz:S.wHz.join('')};ep.t1=t;ep.s1=s;}else if(ep){eps.push(ep);ep=null;}}
  {const p=phase();if(!PH.length||PH[PH.length-1][1]!==p)PH.push([+t.toFixed(1),p]);}
  frame++;}
console.log('phase changes:',JSON.stringify(PH));console.log(tr,G?'PHANTOM':'AUTOPILOT','steer',E.cfg.steer,'drift',E.AP.drift,'cfg',E.cfg.surface,E.cfg.drive,'hazards',E.cfg.hazards,'uniform',E.cfg.uniform,'| laps:',lapsT.join(', '));
for(const e of eps)console.log('  off',f(e.t0),'s, s',f(e.s0,0),'→',f(e.s1,0),'m,',f(e.t1-e.t0,2),'s, at',f(e.v,0),'km/h, β',f(e.b),'°, hazard',e.hz||'-','|',e.ph);
