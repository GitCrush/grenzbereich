// Skidpad tests for the path-relative tutor: false alarms on the line and on a tighter radius, throttle-stab catches,
// and a held power drift in the middle of the pad.
const {loadEngine}=require('./load');const DEG=180/Math.PI;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=1)=>(+x).toFixed(d);
function setup(drive,surface,off){const E=loadEngine();Object.assign(E.cfg,{surface,drive,auto:false,steer:'direct',uniform:true,profile:false,driven:false,tutor:true});E.VERT.amp=1;E.applyDriveSetup(drive);E.selectTrack('kreis');const S=E.S,T=E.TRACK,CAR=E.CAR;
  const p=T.pts[10],q=E.LINE[10];for(const L of E.LINE){L.x+=0;}const shift=off||0;   // shift>0: drive closer to the centre
  if(shift){for(let k=0;k<T.N;k++){const P=T.pts[k],Q=E.LINE[k];Q.x-= -P.nx*0;}}
  S.x=q.x+p.nx*shift;S.y=q.y+p.ny*shift;S.psi=Math.atan2(p.ty,p.tx);S.idx=10;const v0=34/3.6;S.vx=v0;{const w=v0/CAR.Rw;S.w=[w,w,w,w];S.gi=3;S.rpm=w*CAR.gears[3].r*CAR.final*9.55;S.we=S.rpm*0.10472;E.vertInit();}return E;}
// path follower for a given radius around the pad centre (pure pursuit on a circle), so we can choose the radius
function circleSteer(E,R){const S=E.S,T=E.TRACK,CAR=E.CAR;let cxp=0,cyp=0;for(const p of T.pts){cxp+=p.x;cyp+=p.y;}cxp/=T.N;cyp/=T.N;
  const a0=Math.atan2(S.y-cyp,S.x-cxp)+12/R;const tx=cxp+R*Math.cos(a0),ty=cyp+R*Math.sin(a0);const al=Math.atan2(ty-S.y,tx-S.x)-S.psi;const a=Math.atan2(Math.sin(al),Math.cos(al));return Math.atan(2*(CAR.a+CAR.b)*Math.sin(a)/12);}
function run(lab,drive,surface,R,vk,stab,follow,thrHold,lift){const E=setup(drive,surface,0);const S=E.S,CAR=E.CAR;let t=0,sw=0,c={I:0},onT=0,bMax=0,bEnd=0,spun=false,held=0,lastTut=null;
  for(let i=0;i<Math.round(18/E.DT);i++){const vT=(34+(vk-34)*clamp((t-2)/5,0,1))/3.6;const v=Math.hypot(S.vx,S.vy);const e=vT-v;c.I=clamp(c.I+e*E.DT*0.5,-0.5,0.6);let thr=clamp(0.5*e+c.I,0,0.5);
    if(thrHold&&t>9)thr=thrHold;if(stab&&t>10&&t<10+stab)thr=1;
    if(i%6===0)lastTut=E.tutorAdvice();const tut=lastTut;if(lift&&tut&&tut.on&&t>10+(stab||0))thr=Math.min(thr,0.15);E.IN.thr=thr;E.IN.brk=0;
    let target=circleSteer(E,R);if(t<=10)held=target;else if(stab||thrHold)target=held;
    if(follow&&tut&&tut.on){target=tut.target;}if(tut&&tut.on&&t>8)onT+=E.DT;
    const rate=600/DEG/CAR.ratio*E.DT;sw+=clamp(target-sw,-rate,rate);E.IN.steer=clamp(sw/CAR.maxSteer,-1,1);E.step(E.DT);t+=E.DT;
    const b=Math.abs(Math.atan2(S.vy,Math.abs(S.vx))*DEG);if(t>9)bMax=Math.max(bMax,b);if(t>16)bEnd=Math.max(bEnd,b);if(b>75&&t>3){spun=true;break;}}
  console.log(lab.padEnd(52),(spun?'SPIN':'held, beta max '+f(bMax,0)+', last 2 s '+f(bEnd,0)).padEnd(32),'tutor on',f(onT,1),'s');}
run('awd gravel R40 steady 62 km/h','awd','gravel',40,62,0,true);
run('awd gravel R34 (tighter, centre) steady 54','awd','gravel',34,54,0,true);
run('rwd tarmac R40 steady 68','rwd','tarmac',40,68,0,true);
run('rwd gravel R34 power drift thr 0.45, no tutor','rwd','gravel',34,45,0,false,0.45);
run('rwd gravel R34 power drift thr 0.45, follows','rwd','gravel',34,45,0,true,0.45);
run('rwd gravel R40 stab 1 s, no reaction','rwd','gravel',40,52,1.0,false);
run('rwd gravel R40 stab 1 s, follows','rwd','gravel',40,52,1.0,true);
run('rwd tarmac R40 stab 1 s, no reaction','rwd','tarmac',40,68,1.0,false);
run('rwd tarmac R40 stab 1 s, follows','rwd','tarmac',40,68,1.0,true);
run('awd gravel R34 stab 1.5 s, no reaction','awd','gravel',34,54,1.5,false);
run('awd gravel R34 stab 1.5 s, follows','awd','gravel',34,54,1.5,true);
run('rwd tarmac R40 stab 1 s, follows and lifts','rwd','tarmac',40,68,1.0,true,0,true);
run('rwd gravel R40 stab 1 s, follows and lifts','rwd','gravel',40,52,1.0,true,0,true);
