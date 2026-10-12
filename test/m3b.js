// Review M3, robust rig: steady circle R 40 m on tarmac reached slowly (throttle capped, speed ramp), then lift to zero with the
// steering held. Yaw-rate ratio r(0.5 s)/r(0) and rear-axle forces before/after. Variants isolate the candidates.
const {loadEngine}=require('./load');const DEG=180/Math.PI;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=2)=>(+x).toFixed(d);
function run(lab,drive,vk,mod,src){const E=loadEngine(src?{src}:undefined);Object.assign(E.cfg,{surface:'tarmac',drive,auto:true,steer:'direct',uniform:true,profile:false,driven:false,hazards:'off'});E.VERT.amp=0;E.applyDriveSetup(drive);if(mod)mod(E);E.selectTrack('kreis');
  const S=E.S,T=E.TRACK,CAR=E.CAR;const p=T.pts[10],q=E.LINE[10];S.x=q.x;S.y=q.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=10;S.vx=34/3.6;{const w=S.vx/CAR.Rw;S.w=[w,w,w,w];S.gi=3;S.rpm=w*CAR.gears[3].r*CAR.final*9.55;S.we=S.rpm*0.10472;}E.vertInit();
  let cxp=0,cyp=0;for(const pp of T.pts){cxp+=pp.x;cyp+=pp.y;}cxp/=T.N;cyp/=T.N;const c={I:0};let t=0,r0=0,b0=0,held=0,ay0=0,before=null;const L0=12;
  for(let i=0;i<Math.round(16/E.DT);i++){const v=Math.hypot(S.vx,S.vy);
    if(t<14){const vT=(34+(vk-34)*clamp((t-1)/9,0,1))/3.6;const e=vT-v;c.I=clamp(c.I+e*E.DT*0.4,-0.3,0.55);E.IN.thr=clamp(0.35*e+c.I,0,0.6);
      const a0=Math.atan2(S.y-cyp,S.x-cxp)+L0/40*(T.pts[10].ks||1);const tx=cxp+40*Math.cos(a0),ty=cyp+40*Math.sin(a0);let al=Math.atan2(ty-S.y,tx-S.x)-S.psi;al=Math.atan2(Math.sin(al),Math.cos(al));
      E.IN.steer=clamp(Math.atan(2*(CAR.a+CAR.b)*Math.sin(al)/L0)/CAR.maxSteer,-1,1);held=E.IN.steer;r0=S.r;b0=Math.atan2(S.vy,Math.abs(S.vx));ay0=Math.abs(S.ay);
      before=[S.Fxr[2]+S.Fxr[3],S.Fyr[2]+S.Fyr[3],S.Fz[2]+S.Fz[3]];}
    else{E.IN.thr=0;E.IN.steer=held;}
    E.step(E.DT);t+=E.DT;
    if(Math.abs(t-14.5)<E.DT/2){const after=[S.Fxr[2]+S.Fxr[3],S.Fyr[2]+S.Fyr[3],S.Fz[2]+S.Fz[3]];
      console.log(lab.padEnd(34),'ay',f(ay0/9.81),'g | beta',f(b0*DEG,1),'deg | yaw ratio 0.5 s',f(S.r/r0),'| rear Fx',Math.round(before[0]),'->',Math.round(after[0]),'N, Fy',Math.round(before[1]),'->',Math.round(after[1]),', Fz',Math.round(before[2]),'->',Math.round(after[2]));}}}

for(const vk of [72,76])run('RWD base '+vk+' km/h','rwd',vk);
run('RWD 74, coast lock 0','rwd',74,E=>{E.cfg.lsdCoast=0;});
run('RWD 74, coast lock 90','rwd',74,E=>{E.cfg.lsdCoast=0.9;});
run('RWD 74, roll 43 % front (AWD value)','rwd',74,E=>{E.cfg.roll=0.43;});
run('RWD 74, roll 60 % front','rwd',74,E=>{E.cfg.roll=0.60;});
run('RWD 74, anti-lag off','rwd',74,E=>{E.cfg.antilag=false;});
run('RWD 74, coast 25 %, roll 50 %','rwd',74,E=>{E.cfg.lsdCoast=0.25;E.cfg.roll=0.50;});
run('RWD 74, coast 25 %, roll 48 %','rwd',74,E=>{E.cfg.lsdCoast=0.25;E.cfg.roll=0.48;});
run('RWD 70, coast 25 %, roll 48 %','rwd',70,E=>{E.cfg.lsdCoast=0.25;E.cfg.roll=0.48;});
run('RWD 74, rear toe 0.10, roll steer 0','rwd',74,E=>{E.cfg.toeR=0.10;E.cfg.rollSteerR=0;});
run('RWD 74, rear toe 0.15, roll steer 0.05, coast 25','rwd',74,E=>{E.cfg.toeR=0.15;E.cfg.rollSteerR=0.05;E.cfg.lsdCoast=0.25;});
run('AWD 76 (reference)','awd',76);
