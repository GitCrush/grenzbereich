// Transients on gravel vs tarmac, AWD base setup, 80 km/h: a lane change (one sine of steering, 0.5 Hz) sized for ~0.5 g,
// and a lift-off in a steady 0.55 g corner. Body slip angle and yaw overshoot.
const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=1)=>(+x).toFixed(d);
function mk(surface,driven){const E=loadEngine();Object.assign(E.cfg,{surface,drive:'awd',auto:true,steer:'direct',uniform:!driven,profile:false,driven:!!driven,hazards:'off'});E.VERT.amp=1;E.selectTrack('bremse');const S=E.S,T=E.TRACK,CAR=E.CAR;
  const p=T.pts[30];S.x=p.x;S.y=p.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=30;S.vx=80/3.6;{const w=S.vx/CAR.Rw;S.w=[w,w,w,w];S.gi=4;S.rpm=w*CAR.gears[4].r*CAR.final*9.55;S.we=S.rpm*0.10472;}E.vertInit();return E;}
function lane(surface,amp){const E=mk(surface);const S=E.S,CAR=E.CAR;let t=0,bMax=0,ayMax=0;const c={I:0};
  for(let i=0;i<Math.round(4/E.DT);i++){const e=80/3.6-Math.hypot(S.vx,S.vy);c.I=clamp(c.I+e*E.DT*0.5,-0.5,0.8);E.IN.thr=clamp(0.5*e+c.I,0,1);E.IN.steer=t<2?amp*Math.sin(Math.PI*t):0;E.step(E.DT);t+=E.DT;
    bMax=Math.max(bMax,Math.abs(Math.atan2(S.vy,Math.abs(S.vx)))*DEG);ayMax=Math.max(ayMax,Math.abs(S.ay)/G);}
  return {bMax,ayMax};}
for(const surface of ['tarmac','gravel']){const r=[];for(const amp of [0.03,0.05,0.08]){const x=lane(surface,amp);r.push('ay '+f(x.ayMax,2)+' g -> beta '+f(x.bMax,1)+' deg');}console.log('lane change',surface.padEnd(7),r.join(' | '));}
