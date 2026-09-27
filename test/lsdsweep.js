const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=1)=>(+x).toFixed(d);
function fresh(surface,drive,lsd,coast){const E=loadEngine();Object.assign(E.cfg,{surface,drive,auto:true,steer:'direct',uniform:true,profile:false,driven:false});E.VERT.amp=0;E.applyDriveSetup(drive);E.cfg.lsd=lsd;E.cfg.lsdCoast=coast;E.selectTrack('rundkurs');return E;}
function setSpeed(E,v,gi){const S=E.S,CAR=E.CAR;S.vx=v;S.vy=0;S.r=0;S.delta=0;S.dComp=0;S.psi=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.gi=gi||3;S.rpm=w*CAR.gears[S.gi].r*CAR.final*9.55;S.we=S.rpm*0.10472;S.shiftLock=0.4;E.vertInit();}
function hold(E,vT,c){const e=vT-Math.hypot(E.S.vx,E.S.vy);c.I=clamp((c.I||0)+e*E.DT*0.6,-0.9,0.9);const u=0.35*e+c.I;E.IN.thr=clamp(u,0,1);E.IN.brk=clamp(-u*0.5,0,1);}
function launch(E){const S=E.S;E.IN.thr=1;let t=0;while(t<16){E.step(E.DT);t+=E.DT;if(Math.hypot(S.vx,S.vy)>=100/3.6)return t;}return null;}
function power(E,surface){const S=E.S;const vT=60/3.6;setSpeed(E,vT);const c={};for(let i=0;i<720;i++){hold(E,vT,c);E.step(E.DT);}
  E.IN.steer=((E.CAR.a+E.CAR.b)/(vT*vT)+(2.5/DEG)/G)*0.45*G/E.CAR.maxSteer;for(let i=0;i<1080;i++){hold(E,vT,c);E.step(E.DT);}const r0=S.r;let kIn=0,bMax=0;
  for(let i=0;i<360;i++){E.IN.thr=1;E.IN.brk=0;E.step(E.DT);kIn=Math.max(kIn,S.kappa[2]);bMax=Math.max(bMax,Math.abs(Math.atan2(S.vy,S.vx))*DEG);}return {dr:(S.r/r0-1)*100,kIn,bMax};}
function lift(E,surface){const S=E.S;const vT=60/3.6;setSpeed(E,vT);const c={};for(let i=0;i<720;i++){hold(E,vT,c);E.step(E.DT);}
  E.IN.steer=((E.CAR.a+E.CAR.b)/(vT*vT)+(2.5/DEG)/G)*0.45*G/E.CAR.maxSteer;for(let i=0;i<1080;i++){hold(E,vT,c);E.step(E.DT);}const r0=S.r;let rMax=r0;
  for(let i=0;i<360;i++){E.IN.thr=0;E.IN.brk=0;E.step(E.DT);rMax=Math.max(rMax,S.r);}return (rMax/r0-1)*100;}
function windup(E){const S=E.S;setSpeed(E,20/3.6,2);const c={};E.IN.steer=(14/DEG)/E.CAR.maxSteer;for(let i=0;i<1800;i++){hold(E,20/3.6,c);E.step(E.DT);}return {thr:E.IN.thr,dw:(S.w[1]-S.w[0])*E.CAR.Rw*3.6};}
function catchSlide(E,vk){const S=E.S;setSpeed(E,vk/3.6);let t=0,bMax=0,rec=null;for(let i=0;i<Math.round(4/E.DT);i++){const b=Math.atan2(S.vy,Math.abs(S.vx));
  if(t<0.7){E.IN.steer=0.6;E.IN.thr=1;}else{E.IN.steer=clamp(1.2*b/E.CAR.maxSteer,-1,1);E.IN.thr=0;}E.step(E.DT);t+=E.DT;bMax=Math.max(bMax,Math.abs(b)*DEG);if(t>0.7&&rec===null&&Math.abs(b)*DEG<5&&bMax>10)rec=t-0.7;}return {bMax,rec};}
console.log('Conventional locking value sweep (drive / coast). Old behaviour ≈ 110/60 (AWD), 140/80 (RWD).');
console.log('drive lock   | AWD gravel: 0–100   power-stab Δr  κ inside  lift Δr | AWD tarmac: 0–100  wind-up 20 km/h throttle, Δv l/r | RWD tarmac: power-stab Δr β  catch40 β/t | RWD gravel: power Δr β  catch40 β/t');
for(const [lsd,coast] of [[1.0,0.6],[0.8,0.5],[0.6,0.4],[0.5,0.3],[0.4,0.25],[0.3,0.2],[0.2,0.15]]){
  const a1=launch(fresh('gravel','awd',lsd,coast)),a2=power(fresh('gravel','awd',lsd,coast),'gravel'),a3=lift(fresh('gravel','awd',lsd,coast)),a4=launch(fresh('tarmac','awd',lsd,coast)),a5=windup(fresh('tarmac','awd',lsd,coast));
  const rl=lsd,rc=coast;const r1=power(fresh('tarmac','rwd',rl,rc),'tarmac'),r2=catchSlide(fresh('tarmac','rwd',rl,rc),40),r3=power(fresh('gravel','rwd',rl,rc),'gravel'),r4=catchSlide(fresh('gravel','rwd',rl,rc),40);
  console.log((f(lsd*100,0)+'/'+f(coast*100,0)+' %').padStart(10),'  |',f(a1,2),'s ',f(a2.dr,0).padStart(5),'%',f(a2.kIn,2),'  ',f(a3,0).padStart(4),'% |',f(a4,2),'s  thr',f(a5.thr,2),'Δv',f(a5.dw,1).padStart(5),'km/h |',f(r1.dr,0).padStart(5),'%',f(r1.bMax,0).padStart(3),'°',f(r2.bMax,0).padStart(3),'°/'+(r2.rec!==null?f(r2.rec)+'s':'never'),'|',f(r3.dr,0).padStart(5),'%',f(r3.bMax,0).padStart(3),'°',f(r4.bMax,0).padStart(3),'°/'+(r4.rec!==null?f(r4.rec)+'s':'never'));
}
