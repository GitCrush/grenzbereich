const {loadEngine}=require('./load');const DEG=180/Math.PI;const f=(x,d=1)=>(+x).toFixed(d);
function setSpeed(E,v){const S=E.S,CAR=E.CAR;S.vx=v;S.vy=0;S.r=0;S.delta=0;S.dComp=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.Fxr=[0,0,0,0];S.Fyr=[0,0,0,0];S.gi=3;S.rpm=Math.max(CAR.idle,w*CAR.gears[3].r*CAR.final*9.55);S.we=S.rpm*0.10472;S.shiftLock=0.4;E.vertInit();}
function run(surface,vk,drive,kick,recover,extra){const E=loadEngine();Object.assign(E.cfg,{surface,drive,auto:true,steer:'direct',uniform:true,profile:false,driven:false},extra||{});E.VERT.amp=0;E.selectTrack('rundkurs');const S=E.S;setSpeed(E,vk/3.6);
  let t=0,bMax=0,bEnd=0,recT=null,psiTot=0,vMin=1e9;const tr=[];
  for(let i=0;i<Math.round(4/E.DT);i++){const b=Math.atan2(S.vy,Math.abs(S.vx));
    if(t<kick.dur){E.IN.steer=kick.steer;E.IN.thr=kick.thr;E.IN.brk=0;}
    else{E.IN.steer=Math.max(-1,Math.min(1,recover.k*b/E.CAR.maxSteer));E.IN.thr=recover.thr;E.IN.brk=recover.brk||0;}
    E.step(E.DT);t+=E.DT;psiTot=Math.max(psiTot,Math.abs(S.psi-0));bMax=Math.max(bMax,Math.abs(b)*DEG);vMin=Math.min(vMin,Math.hypot(S.vx,S.vy));
    if(t>kick.dur&&recT===null&&Math.abs(b)*DEG<5&&bMax>10)recT=t;
    if(Math.round(t/E.DT)%72===0)tr.push(f(t)+':'+f(b*DEG,0)+'/κ'+f(S.kappa[2],1));}
  const b=Math.atan2(S.vy,Math.abs(S.vx))*DEG;
  return {bMax,bEnd:b,recT,tr:tr.join(' ')};}
console.log('Engine:',process.env.ENGINE||'engine.js');
const kick={dur:0.7,steer:0.6,thr:1};
for(const [surface,vk] of [['gravel',40],['gravel',60],['tarmac',40],['tarmac',60]]){
  for(const [lab,rec] of [['counter-steer 1.2×β, throttle 0',{k:1.2,thr:0}],['counter-steer 1.2×β, throttle 0.3',{k:1.2,thr:0.3}],['counter-steer 1.5×β, throttle 0, clutch',{k:1.5,thr:0,clutch:true}]]){
    const extra={};const E0=null;
    const r=run(surface,vk,'rwd',kick,rec);
    console.log((surface+' '+vk+' km/h RWD, '+lab).padEnd(46),'β max',f(r.bMax,0).padStart(3),'°  end',f(r.bEnd,0).padStart(4),'°  caught after',r.recT?f(r.recT-kick.dur)+' s':'never','   '+r.tr);
  }
}
