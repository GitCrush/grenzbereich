// Steady cornering on the skidpad (R 40 m) at rising speed: lateral acceleration, body slip angle, front and rear tyre
// slip angles and usage – gravel vs tarmac, AWD, base setup. Plus the tyre's own curve on gravel.
const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=1)=>(+x).toFixed(d);
function run(surface,drive){const E=loadEngine();Object.assign(E.cfg,{surface,drive,auto:true,steer:'direct',uniform:true,profile:false,driven:false,hazards:'off'});E.VERT.amp=1;E.applyDriveSetup(drive);E.selectTrack('kreis');const S=E.S,T=E.TRACK,CAR=E.CAR;
  const p=T.pts[10],q=E.LINE[10];S.x=q.x;S.y=q.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=10;S.vx=30/3.6;{const w=S.vx/CAR.Rw;S.w=[w,w,w,w];S.gi=3;}E.vertInit();
  let t=0,c={I:0};const rows=[];let next=35;
  for(let i=0;i<Math.round(60/E.DT);i++){const vT=(30+0.6*t)/3.6;const v=Math.hypot(S.vx,S.vy);const e=vT-v;c.I=clamp(c.I+e*E.DT*0.5,-0.5,0.8);E.IN.thr=clamp(0.5*e+c.I,0,1);E.IN.brk=0;
    E.IN.steer=E.lineSteer().delta/CAR.maxSteer;E.step(E.DT);t+=E.DT;const kmh=v*3.6;
    if(kmh>=next){const b=Math.atan2(S.vy,Math.abs(S.vx))*DEG;rows.push(f(kmh,0)+' km/h: ay '+f(Math.abs(S.ay)/G,2)+' g, beta '+f(b,1)+', alpha f/r '+f(Math.abs((S.alpha[0]+S.alpha[1])/2)*DEG,1)+'/'+f(Math.abs((S.alpha[2]+S.alpha[3])/2)*DEG,1)+' deg, usage f/r '+f(Math.max(S.slip[0],S.slip[1]),2)+'/'+f(Math.max(S.slip[2],S.slip[3]),2));next+=5;}
    if(Math.abs(Math.atan2(S.vy,Math.abs(S.vx)))>0.6||next>75)break;}
  console.log(surface,drive);console.log('  '+rows.join('\n  '));}
run('gravel','awd');run('tarmac','awd');
const E=loadEngine();const sur=E.SURF.gravel;console.log('gravel tyre: peak slip angle',f(sur.aP*DEG,1),'deg, peak slip',sur.kP,', force beyond the peak falls to',sur.tail);
