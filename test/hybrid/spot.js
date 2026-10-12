// One stretch of a lap, traced (decision 11 Oct: stage 4711 goes off at s ≈ 1642 m with the controller on, 800 m after its
// corner): every 0.1 s between S0 and S1 on lap 2 – slip, the slide wanted, speed and plan, offset and the road's half
// width, steering, throttle, brake, the catch. node hybrid/spot.js S0 S1   env: SEED/TRACK, DRIVE, SETA, LSFILE
const {loadEngine}=require('../load');const f=(x,d=1)=>(x>=0?' ':'')+(+x).toFixed(d);const E=loadEngine();if(process.env.DRIVE){E.cfg.drive=process.env.DRIVE;E.applyDriveSetup(E.cfg.drive);}
E.AP.drift=+(process.env.DRIFT||1);if(process.env.SETA)Object.assign(E.AP,JSON.parse(process.env.SETA));
if(process.env.SEED){E.TRACKS.random=E.randomTrackDef(+process.env.SEED);E.selectTrack('random');}else E.selectTrack(process.env.TRACK||'rundkurs');
E.AP.on=true;E.apPrepare();const S=E.S,T=E.TRACK,L=T.pts[T.N-1].s,DT=E.DT,s0=+process.argv[2],s1=+process.argv[3];let t=0,laps=0,prev=T.pts[S.idx].s,n=0;
while(laps<2&&t<500){E.IN.steer=0;E.IN.thr=0;E.IN.brk=0;E.IN.hb=0;if(n%180===0)E.apEnsure();const c=E.ghostCommand(E.AP.pedBrk||0,DT,'catch');E.AP.pedBrk=c.brk;E.IN.steer=c.steer;E.IN.thr=c.thr;E.IN.brk=c.brk;E.IN.hb=c.hb||0;E.step(DT);t+=DT;n++;
  const s=T.pts[S.idx].s;if(s<prev-L/2)laps++;prev=s;if(laps!==1||s<s0||s>s1||n%36)continue;const P=T.pts[S.idx],lat=(S.x-P.x)*P.nx+(S.y-P.y)*P.ny;
  console.log(`s ${f(s,0)} β${f(Math.atan2(S.vy,Math.abs(S.vx))*57.3)} want${f((E.AP.brNow||0)*57.3)} v${f(Math.hypot(S.vx,S.vy)*3.6,0)} plan${f(E.AP.prof.v[S.idx]*3.6,0)} lat${f(lat)}/${f(P.w)} line${f(E.AP.line?E.AP.line[S.idx].o:0)} r${f(S.r*57.3)} δ${f(S.delta*57.3)} thr${f(E.IN.thr,2)} brk${f(E.IN.brk,2)} R${f(E.AP.line?E.AP.line[S.idx].R:0,0)}${E.AP.dcS&&E.AP.dcS.on?' DC':''}${S.onTrack?'':' OFF'}`);}
