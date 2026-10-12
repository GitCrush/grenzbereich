// Review M3: lift-off in a steady corner (tarmac, R 40 m, ~1.1 g), steering held: yaw-rate ratio after 0.5 s and slip-angle change.
// Variants isolate the candidates: coast lock, roll-stiffness distribution, engine drag.
const {loadEngine}=require('./load');const DEG=180/Math.PI;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=2)=>(+x).toFixed(d);
function run(lab,drive,mod){const E=loadEngine();Object.assign(E.cfg,{surface:'tarmac',drive,auto:true,steer:'direct',uniform:true,profile:false,driven:false,hazards:'off'});E.VERT.amp=0;E.applyDriveSetup(drive);if(mod)mod(E);E.selectTrack('kreis');
  const S=E.S,T=E.TRACK,CAR=E.CAR;const p=T.pts[10],q=E.LINE[10];S.x=q.x;S.y=q.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=10;S.vx=20;{const w=S.vx/CAR.Rw;S.w=[w,w,w,w];S.gi=3;S.rpm=w*CAR.gears[3].r*CAR.final*9.55;S.we=S.rpm*0.10472;}E.vertInit();
  let cxp=0,cyp=0;for(const pp of T.pts){cxp+=pp.x;cyp+=pp.y;}cxp/=T.N;cyp/=T.N;const c={I:0};let t=0,r0=0,b0=0,held=0;
  for(let i=0;i<Math.round(14/E.DT);i++){const vT=(20+(75-20)*clamp(t/9,0,1))/3.6*3.6/3.6;const v=Math.hypot(S.vx,S.vy);
    if(t<10){const vTgt=Math.min(23.5,15+t);const e=vTgt-v;c.I=clamp(c.I+e*E.DT*0.5,-0.5,0.9);E.IN.thr=clamp(0.5*e+c.I,0,1);
      const a0=Math.atan2(S.y-cyp,S.x-cxp)+12/40*(T.pts[10].ks||1);const tx=cxp+40*Math.cos(a0),ty=cyp+40*Math.sin(a0);let al=Math.atan2(ty-S.y,tx-S.x)-S.psi;al=Math.atan2(Math.sin(al),Math.cos(al));
      E.IN.steer=clamp(Math.atan(2*(CAR.a+CAR.b)*Math.sin(al)/12)/CAR.maxSteer,-1,1);held=E.IN.steer;r0=S.r;b0=Math.atan2(S.vy,Math.abs(S.vx));}
    else{E.IN.thr=0;E.IN.steer=held;}
    E.step(E.DT);t+=E.DT;if(Math.abs(t-10.5)<E.DT/2){console.log(lab.padEnd(40),'ay before',f(Math.abs(S.ay)/9.81),'g | yaw ratio after 0.5 s',f(S.r/r0),'| slip angle',f(b0*DEG,1),'->',f(Math.atan2(S.vy,Math.abs(S.vx))*DEG,1),'deg');}}}
run('AWD base','awd');
run('RWD base (coast lock 45 %)','rwd');
run('RWD coast lock 0','rwd',E=>{E.cfg.lsdCoast=0;});
run('RWD coast lock 90 %','rwd',E=>{E.cfg.lsdCoast=0.9;});
run('RWD roll stiffness 40 % front','rwd',E=>{E.cfg.roll=0.40;});
run('RWD roll stiffness 65 % front','rwd',E=>{E.cfg.roll=0.65;});
run('RWD brake bias irrelevant check (60 %)','rwd',E=>{E.cfg.bias=0.6;});
