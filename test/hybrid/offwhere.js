// Where does the autopilot leave the road? (decision 11 Oct: stage 4711 with the controller on, 0.3 s off – not in its
// corner). Drives as drift-bench.js does (inputs reset every frame, the plan first), lap 2; per off-road episode: time,
// track position, slip, the slide wanted, whether the controller drives, the corner it belongs to, speed against the plan.
// node hybrid/offwhere.js   env: SEED / TRACK / OVAL, DRIVE, DRIFT (default 1), SETA, LSFILE
const {loadEngine}=require('../load');const f=(x,d=1)=>(+x).toFixed(d);const E=loadEngine();if(process.env.DRIVE){E.cfg.drive=process.env.DRIVE;E.applyDriveSetup(E.cfg.drive);}
E.AP.drift=+(process.env.DRIFT||1);if(process.env.SETA)Object.assign(E.AP,JSON.parse(process.env.SETA));
if(process.env.SEED){E.TRACKS.random=E.randomTrackDef(+process.env.SEED);E.selectTrack('random');}else E.selectTrack(process.env.TRACK||'rundkurs');
E.AP.on=true;E.apPrepare();const S=E.S,T=E.TRACK,L=T.pts[T.N-1].s,DT=E.DT,C=E.AP.dcC;let t=0,laps=0,prev=T.pts[S.idx].s,n=0,off=null;
console.log('controller corners',C?C.segs.map(([a,b])=>`${f(T.pts[a].s,0)}–${f(T.pts[b].s,0)} m`).join(', '):'none');
while(laps<2&&t<500){E.IN.steer=0;E.IN.thr=0;E.IN.brk=0;E.IN.hb=0;if(n%180===0)E.apEnsure();const c=E.ghostCommand(E.AP.pedBrk||0,DT,'catch');E.AP.pedBrk=c.brk;E.IN.steer=c.steer;E.IN.thr=c.thr;E.IN.brk=c.brk;E.IN.hb=c.hb||0;E.step(DT);t+=DT;n++;
  const s=T.pts[S.idx].s;if(s<prev-L/2)laps++;prev=s;if(laps<1)continue;
  if(!S.onTrack&&!off){const P=T.pts[S.idx],lat=(S.x-P.x)*P.nx+(S.y-P.y)*P.ny;off={t,s,b:Math.atan2(S.vy,Math.abs(S.vx))*57.3,br:(E.AP.brNow||0)*57.3,dc:!!(E.AP.dcS&&E.AP.dcS.on),ab:!!E.AP.dcAb,inC:!!(C&&C.dc[S.idx]),v:Math.hypot(S.vx,S.vy)*3.6,vp:E.AP.prof?E.AP.prof.v[S.idx]*3.6:0,lat,w:P.w};}
  if(S.onTrack&&off){console.log(`off ${f(t-off.t,2)} s at ${f(off.t)} s, s ${f(off.s,0)} m: β ${f(off.b)}° wanted ${f(off.br)}° | controller ${off.dc?'on':'off'}${off.ab?' (given up)':''}, in its corner ${off.inC} | v ${f(off.v,0)} plan ${f(off.vp,0)} | lat ${f(off.lat)} of ±${f(off.w)}`);off=null;}}
