// RWD: power-induced slide at 60 km/h on tarmac and gravel, then human-like counter-steer with reaction time and limited hand speed.
const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=1)=>(+x).toFixed(d);
function fresh(surface,mod){const E=loadEngine();Object.assign(E.cfg,{surface,drive:'rwd',auto:true,steer:'direct',uniform:true,profile:false,driven:false});E.VERT.amp=0;E.applyDriveSetup('rwd');mod&&mod(E);E.selectTrack('rundkurs');return E;}
function setSpeed(E,v,gi){const S=E.S,CAR=E.CAR;S.vx=v;S.vy=0;S.r=0;S.delta=0;S.dComp=0;S.psi=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.gi=gi||3;S.rpm=w*CAR.gears[S.gi].r*CAR.final*9.55;S.we=S.rpm*0.10472;S.shiftLock=0.4;E.vertInit();}
function hold(E,vT,c){const e=vT-Math.hypot(E.S.vx,E.S.vy);c.I=clamp((c.I||0)+e*E.DT*0.6,-0.9,0.9);const u=0.35*e+c.I;E.IN.thr=clamp(u,0,1);E.IN.brk=clamp(-u*0.5,0,1);}
function run(lab,surface,drv,mod){const E=fresh(surface,mod);const S=E.S,CAR=E.CAR;const vT=60/3.6;setSpeed(E,vT);const c={};for(let i=0;i<720;i++){hold(E,vT,c);E.step(E.DT);}
  const ayT=0.45;E.IN.steer=((CAR.a+CAR.b)/(vT*vT)+(2/DEG)/G)*ayT*G/CAR.maxSteer;for(let i=0;i<1080;i++){hold(E,vT,c);E.step(E.DT);}
  const steer0=E.IN.steer;let t=0,bMax=0,rec=null,tOut=null,sw=E.IN.steer,rpmMin=1e9,spun=false;const o=[];
  for(let i=0;i<Math.round(4/E.DT);i++){const b=Math.atan2(S.vy,Math.abs(S.vx));const bd=b*DEG;
    if(t<0.5){E.IN.thr=1;E.IN.steer=steer0;}
    else{if(tOut===null&&Math.abs(bd)>drv.trig)tOut=t;
      if(tOut!==null&&t>tOut+drv.react){const target=clamp(drv.gain*b/CAR.maxSteer,-drv.max/(CAR.lock/2),drv.max/(CAR.lock/2));const rate=drv.rate/(CAR.lock/2)*E.DT;sw+=clamp(target-sw,-rate,rate);E.IN.steer=sw;E.IN.thr=drv.thr;}
      else{E.IN.steer=steer0;E.IN.thr=1;}}
    E.step(E.DT);t+=E.DT;bMax=Math.max(bMax,Math.abs(bd));rpmMin=Math.min(rpmMin,S.rpm);
    if(tOut!==null&&rec===null&&Math.abs(bd)<5&&bMax>10&&t>tOut+0.3)rec=t-tOut;if(Math.abs(bd)>90)spun=true;
    if(i%72===0&&t>0.4)o.push(f(t,1)+':b'+f(bd,0)+'/sw'+f(sw*CAR.lock/2,0)+'/rpm'+f(S.rpm,0));}
  console.log(lab.padEnd(58),'beta max',f(bMax,0).padStart(3),spun?'SPIN':(rec!==null?'caught after '+f(rec)+' s':'not caught'),' rpm min',f(rpmMin,0),'  ',o.slice(0,9).join(' '));}
const ideal={trig:3,react:0.0,gain:1.2,max:400,rate:5000,thr:0};
const human={trig:5,react:0.25,gain:1.2,max:300,rate:500,thr:0};
const humanThr={trig:5,react:0.25,gain:1.2,max:300,rate:500,thr:0.3};
const slow={trig:8,react:0.35,gain:1.2,max:250,rate:350,thr:0};
for(const surface of ['tarmac','gravel']){
  run(surface+': ideal (instant, 1.2*beta, throttle off)',surface,ideal);
  run(surface+': human (0.25 s, 500 deg/s, max 300 deg, throttle off)',surface,human);
  run(surface+': human, keeps 30 % throttle',surface,humanThr);
  run(surface+': slow human (0.35 s, 350 deg/s, max 250 deg)',surface,slow);
  run(surface+': human, drift template',surface,human,E=>E.applySetupTemplate('RWD drift'));
}
