const {loadEngine}=require('./load');const DEG=180/Math.PI;const f=(x,d=1)=>(+x).toFixed(d);
function setSpeed(E,v){const S=E.S,CAR=E.CAR;S.vx=v;S.vy=0;S.r=0;S.delta=0;S.dComp=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.Fxr=[0,0,0,0];S.Fyr=[0,0,0,0];S.gi=3;S.rpm=w*CAR.gears[3].r*CAR.final*9.55;S.we=S.rpm*0.10472;S.shiftLock=0.4;E.vertInit();}
function run(surface,vk,steer,hbT,thrAfter,mod){const E=loadEngine();Object.assign(E.cfg,{surface,auto:true,steer:'direct',uniform:true,profile:false});if(mod)mod(E);E.VERT.amp=0;E.selectTrack('rundkurs');const S=E.S;setSpeed(E,vk/3.6);
  let t=0,rMax=0,bMax=0,bEnd=0,b1=0,kR=0;const o=[];
  for(let i=0;i<Math.round(2.2/E.DT);i++){const hb=t<hbT?1:0;E.IN.steer=steer;E.IN.thr=t<hbT?0:thrAfter;E.IN.hb=hb;E.IN.brk=0;E.step(E.DT);t+=E.DT;
    const b=Math.atan2(S.vy,Math.abs(S.vx))*DEG;rMax=Math.max(rMax,Math.abs(S.r)*DEG);bMax=Math.max(bMax,Math.abs(b));if(Math.abs(t-hbT)<E.DT/2)b1=b;if(Math.abs(t-1.2)<E.DT/2)bEnd=b;
    if(Math.round(t/E.DT)%72===0)o.push(f(t)+':β'+f(b,0)+'/r'+f(S.r*DEG,0)+'/wR'+f(S.w[2]*E.CAR.Rw*3.6,0));}
  return {rMax,bMax,bAtRelease:b1,b12:bEnd,trace:o.join(' ')};}
const cases=[['gravel',60,0.3,0.6,0],['gravel',60,0.3,0.6,0.6],['tarmac',50,0.3,0.6,0],['gravel',80,0.2,0.5,0.5]];
console.log('Engine:',process.env.ENGINE||'engine.js');
for(const c of cases){const r=run(...c);console.log(c.join('/').padEnd(24),'r max',f(r.rMax,0).padStart(3),'°/s  β max',f(r.bMax,0).padStart(3),'°  β at release',f(r.bAtRelease,0).padStart(3),'°  β at 1.2 s',f(r.b12,0).padStart(4),'°   ',r.trace);}
if(!process.env.ENGINE){
 console.log('--- Isolating causes (gravel 60 km/h, steer 0.3, HB 0.6 s, then throttle 0.6) ---');
 for(const [lab,mod] of [['default',null],['kinematics off (toe, roll steer 0)',E=>Object.assign(E.CAR,{toeF:0,toeR:0,bsF:0,bsR:0})],['coast lock rear 0',E=>E.cfg.lsdCoast=0],['centre 50 % plated',E=>E.cfg.clock=0.5],['camber coefficient 0.45 – info only',null],['roll distribution 35 % front',E=>E.cfg.roll=0.35]]){
   const r=run('gravel',60,0.3,0.6,0.6,mod);console.log('  '+lab.padEnd(40),'r max',f(r.rMax,0),'  β max',f(r.bMax,0),'  β at release',f(r.bAtRelease,0),'  β 1.2 s',f(r.b12,0));}
}
