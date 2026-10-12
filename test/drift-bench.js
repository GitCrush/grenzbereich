// The autopilot's slides, measured as the browser drives (inputs reset every frame, plan computed first), over the lap
// after the first. node drift-bench.js track drift   env: SEED (random stage), OVAL=R,straight (a stadium oval, e.g.
// 83,250), DRIVE, SURF, REACT (driver reaction 0..1), SETA / SETC / SETS (JSON for AP, CAR, the gravel surface), NL, TAG.
// Prints: lap; on the stretches where a slide is planned – the share driven sideways (over 6°, the planned side), within
// ±4° of the target, rms error; slides (episodes over 6°) – count, mean and longest duration; the largest slip angle;
// time off the road; visible steering corrections per second (hands, and the steering wheel itself, reversing by > 10°).
const {loadEngine}=require('./load');const f=(x,d=1)=>(+x).toFixed(d);
const tr=process.argv[2]||'rundkurs',drift=+(process.argv[3]||1),nl=+(process.env.NL||1);
const E=loadEngine();if(process.env.DRIVE){E.cfg.drive=process.env.DRIVE;E.applyDriveSetup(E.cfg.drive);}if(process.env.SURF)E.cfg.surface=process.env.SURF;E.AP.drift=drift;
if(process.env.REACT)E.applyReaction(+process.env.REACT);if(process.env.SETS)Object.assign(E.SURF.gravel,JSON.parse(process.env.SETS));if(process.env.SETC)Object.assign(E.CAR,JSON.parse(process.env.SETC));if(process.env.SETA)Object.assign(E.AP,JSON.parse(process.env.SETA));
if(process.env.OVAL){const [oR,oS]=process.env.OVAL.split(',').map(Number);E.TRACKS.oval={name:'Oval',type:'seg',segs:[{s:oS},{R:oR,a:180},{s:oS},{R:oR,a:180}],w:10.5,ex:null};E.selectTrack('oval');}
else if(process.env.SEED){E.TRACKS.random=E.randomTrackDef(+process.env.SEED);E.selectTrack('random');}else E.selectTrack(tr);
if(process.env.UNI)E.cfg.uniform=true;if(process.env.NODRIVEN)E.cfg.driven=false;E.AP.on=true;if(E.apPrepare)E.apPrepare();else{E.apEnsure();while(E.AP.job)E.apRun(1e9);E.apEnsure();while(E.AP.lineJob)E.apLineRun(1e9);E.apEnsure();E.AP.lineOn=true;E.AP.profKey="";E.apEnsure();}
const S=E.S,T=E.TRACK,L=T.pts[T.N-1].s,R=E.CAR.ratio,DT=E.DT;let t=0,laps=0,prev=T.pts[S.idx].s,n=0,tL=0;
const M={tp:0,sl:0,inb:0,bmax:0,off:0,lap:[],hRev:0,wRev:0,tAll:0,ep:[],epc:0,b2:0};const rv=(o,x,th)=>{if(o.e===undefined){o.e=x;o.d=0;return 0;}const dd=x-o.e;if(o.d>=0&&dd<-th){o.d=-1;o.e=x;return 1;}if(o.d<=0&&dd>th){o.d=1;o.e=x;return 1;}if((o.d>0&&x>o.e)||(o.d<0&&x<o.e))o.e=x;return 0;};const H={},W={};
while(t<200*(nl+1)&&laps<nl+1){E.IN.steer=0;E.IN.thr=0;E.IN.brk=0;E.IN.hb=0;if(n%180===0)E.apEnsure();
  const c=E.ghostCommand(E.AP.pedBrk||0,DT,'catch');E.AP.pedBrk=c.brk;E.IN.steer=c.steer;E.IN.thr=c.thr;E.IN.brk=c.brk;E.IN.hb=c.hb||0;E.step(DT);t+=DT;n++;tL+=DT;
  const s=T.pts[S.idx].s;if(s<prev-L/2){if(laps>0)M.lap.push(f(tL,1));laps++;tL=0;}prev=s;
  if(laps>=1){M.tAll+=DT;const b=Math.atan2(S.vy,Math.abs(S.vx))*57.3,br=(E.AP.brNow||0)*57.3;if(!S.onTrack){M.off+=DT;if(M.offS===undefined)M.offS=T.pts[S.idx].s;}M.bmax=Math.max(M.bmax,Math.abs(b));
    {const h1=rv(H,(S.hand||0)*57.3,10);M.hRev+=h1;if(h1&&process.env.REVLOG){(M.rl=M.rl||[]).push([t,T.pts[S.idx].s,Math.atan2(S.vy,Math.abs(S.vx))*57.3,E.AP.drOn?1:0,E.AP.dcS&&E.AP.dcS.on?1:0,E.AP.lcS&&E.AP.lcS.cmd!==undefined?1:0,(E.AP.wC||0)]);}}M.wRev+=rv(W,S.delta*R*57.3,10);
    if(Math.abs(b)>6){M.epc+=DT;}else if(M.epc>0){M.ep.push(M.epc);M.epc=0;}
    if(Math.abs(br)>5){M.tp+=DT;if(Math.abs(b)>6&&Math.sign(b)===Math.sign(br))M.sl+=DT;if(Math.abs(b-br)<4)M.inb+=DT;M.b2+=(b-br)*(b-br)*DT;M.bs=(M.bs||0)+Math.abs(b)*DT;M.brs=(M.brs||0)+Math.abs(br)*DT;}}}
if(M.epc>0)M.ep.push(M.epc);const ep=M.ep.filter(x=>x>0.15);
if(process.env.REVLOG&&M.rl){const c={};for(const r of M.rl){const k=r[4]?'controller':r[3]?'older slide':r[6]>0?'catch':r[5]?'line controller':'line law';c[k]=(c[k]||0)+1;}console.log('hand reversals by who steered:',JSON.stringify(c));if(process.env.REVLOG==='2')for(const r of M.rl)console.log('  rev t',r[0].toFixed(2),'s',r[1].toFixed(0),'β',r[2].toFixed(1),'drOn',r[3],'dc',r[4],'lc',r[5],'wC',r[6].toFixed(2));}   /* REVLOG=1: the reversals counted by who steered at the time (12 Oct); 2: each one */
console.log((process.env.TAG||'')+' '+(process.env.OVAL?'oval '+process.env.OVAL:process.env.SEED?'seed '+process.env.SEED:tr),E.cfg.drive,drift,'| laps',M.lap.join(','),'| planned',f(M.tp,1),'s: sliding',f(100*M.sl/Math.max(1e-9,M.tp),0)+'%, ±4°',f(100*M.inb/Math.max(1e-9,M.tp),0)+'%, rms',f(Math.sqrt(M.b2/Math.max(1e-9,M.tp)),1)+'° | slides',ep.length,'mean',f(ep.reduce((a,x)=>a+x,0)/Math.max(1,ep.length),2),'s, longest',f(Math.max(0,...ep),2),'| max',f(M.bmax,0)+'° | off',f(M.off,1)+(M.offS!==undefined?' (from s '+f(M.offS,0)+')':''),'| hand rev/s',f(M.hRev/M.tAll,2),'wheel',f(M.wRev/M.tAll,2));
