// The autopilot's slides at a limit-range setting: held angle against target where a slide is planned, handbrake use,
// excursions, lap. Browser-like (per-frame input reset, plan computed first). node drstat.js track drift [laps=2]   env: SEED, DRIVE, SURF, JIT
const {loadEngine}=require('./load');const f=(x,d=1)=>(+x).toFixed(d);const [tr,drift,nl]=[process.argv[2],+process.argv[3],+(process.argv[4]||2)];
const E=loadEngine();if(process.env.SURF)E.cfg.surface=process.env.SURF;if(process.env.DRIVE){E.cfg.drive=process.env.DRIVE;E.applyDriveSetup(E.cfg.drive);}E.AP.drift=drift;
if(process.env.SETUP)E.applySetupTemplate(process.env.SETUP);   /* a setup template, then single settings: SETX='{"cfg":{"bias":0.62},"CAR":{"h":0.66},"VERT":{"dmpScale":0.7}}' */
if(process.env.SETC)Object.assign(E.CAR,JSON.parse(process.env.SETC));if(process.env.SETA)Object.assign(E.AP,JSON.parse(process.env.SETA));   /* SETC/SETA: CAR and AP fields (driver model, controller trials) */
if(process.env.SETX){const X=JSON.parse(process.env.SETX);for(const o of ['cfg','CAR','VERT'])if(X[o])Object.assign(E[o],X[o]);}
if(process.env.SEED){E.TRACKS.random=E.randomTrackDef(+process.env.SEED);E.selectTrack('random');}else E.selectTrack(tr);
E.AP.on=true;E.apPrepare();
if(process.env.JIT){const P=E.TRACK.pts[E.S.idx],d=0.05*+process.env.JIT;E.S.x+=P.nx*d;E.S.y+=P.ny*d;}   /* JIT=k: the start k × 5 cm to the side – another realisation of the same run, for how robust a case on the road is (11 Oct: two cases' outcome changed with any small change upstream) */
const S=E.S,T=E.TRACK,L=T.pts[T.N-1].s;let t8=0,t12=0,bMx=0;const EPs=[],HIST=[];let EPc=null,mMin=1e9,mS=0;let t=0,laps=0,prev=T.pts[S.idx].s,tl=0,n=0,bS=0,rS=0,nb=0,hbT=0,hbOn=0,off=0,catchT=0;const lapT=[];
while(t<200*(nl+1)&&laps<nl+1){   /* the time cap covers the lap before the measured ones (12 Oct: it was 200 s × laps – on stage 19029, a 165 s lap, only the first 35 s of the measured lap were judged) */
E.IN.steer=0;E.IN.thr=0;E.IN.brk=0;E.IN.hb=0;if(n%180===0)E.apEnsure();
  const c=E.ghostCommand(E.AP.pedBrk||0,E.DT,'catch');E.AP.pedBrk=c.brk;E.IN.steer=c.steer;E.IN.thr=c.thr;E.IN.brk=c.brk;E.IN.hb=c.hb||0;E.step(E.DT);t+=E.DT;tl+=E.DT;n++;
  const s=T.pts[S.idx].s;if(s<prev-L/2){if(laps>0)lapT.push(f(tl));laps++;tl=0;}prev=s;
  if(laps>=1){const br=Math.abs(E.AP.brNow||0);if(br>5/57.3){bS+=Math.abs(Math.atan2(S.vy,Math.abs(S.vx)));rS+=br;nb++;}{const ba=Math.abs(Math.atan2(S.vy,Math.abs(S.vx)))*57.3;if(ba>8)t8+=E.DT;if(ba>12)t12+=E.DT;bMx=Math.max(bMx,ba);}if(c.hb){hbT+=E.DT;if(!hbOn)hbOn=1;}
    {const P=T.pts[S.idx],m=P.w-Math.abs((S.x-P.x)*P.nx+(S.y-P.y)*P.ny);if(m<mMin){mMin=m;mS=s;}}   /* closest approach of the car's centre to the road's edge: how near a case on the road came to failing (11 Oct) */
    if(!S.onTrack)off+=E.DT;if((E.AP.wC||0)>0)catchT+=E.DT;
    if(process.env.EP){if(!S.onTrack){if(!EPc)EPc={s0:s,t0:t,v:Math.hypot(S.vx,S.vy)*3.6,vA:c.vAllow*3.6,b:Math.atan2(S.vy,Math.abs(S.vx))*57.3,br:(E.AP.brNow||0)*57.3,R:E.AP.line[S.idx].R,hist:HIST.slice(-12)};EPc.s1=s;EPc.t1=t;}else if(EPc){EPs.push(EPc);EPc=null;}}}
  if(process.env.EP&&n%36===0){HIST.push([f(T.pts[S.idx].s,0),f(Math.hypot(S.vx,S.vy)*3.6,0),f(c.vAllow*3.6,0),f((E.AP.brNow||0)*57.3),f(Math.atan2(S.vy,Math.abs(S.vx))*57.3),f(c.thr,2),f(c.brk,2),c.hb||0,f(E.AP.line[S.idx].R,0),f(T.pts[S.idx].w-Math.abs((S.x-T.pts[S.idx].x)*T.pts[S.idx].nx+(S.y-T.pts[S.idx].y)*T.pts[S.idx].ny))].join(' '));if(HIST.length>40)HIST.shift();}}
const env=E.AP.env;
if(process.env.EP)for(const e of EPs){console.log('OFF s',f(e.s0,0),'→',f(e.s1,0),f(e.t1-e.t0,2),'s v',f(e.v,0),'plan',f(e.vA,0),'β',f(e.b),'target',f(e.br),'R',f(e.R,0));console.log('   before (s v plan target β thr brk hb R edge):\n   '+e.hist.join('\n   '));}
console.log(tr,E.cfg.surface,E.cfg.drive,'setup',(process.env.SETUP||'default')+(process.env.SETX?' '+process.env.SETX:''),'drift',drift,'| laps',lapT.join(', '),'| held',f(bS/Math.max(1,nb)*57.3),'° of',f(rS/Math.max(1,nb)*57.3),'° | sideways >8° '+f(t8/Math.max(1,lapT.length),1)+' s/lap, >12° '+f(t12/Math.max(1,lapT.length),1)+' s/lap, max '+f(bMx,0)+'° | handbrake',f(hbT,1),'s/lap',f(hbT/Math.max(1,lapT.length),1),'| off',f(off),'s, edge',f(mMin),'m at s',f(mS,0),'| catch',f(catchT),'s | driftMap',JSON.stringify(env.driftMap&&env.driftMap.map(m=>[m[0],+(m[1]*57.3).toFixed(1)])));
