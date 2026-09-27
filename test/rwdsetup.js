// Compare RWD setups: old default, new RWD default, drift setup.
const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=1)=>(+x).toFixed(d);
const SETUPS={
  'old default (AWD values)':{lsd:0.55,lsdCoast:0.30,roll:0.43,toeR:0.15,rs:0.10},
  'new RWD default':null,
  'drift setup':{lsd:0.85,lsdCoast:0.50,roll:0.53,toeR:0.35,rs:0.18}};
function fresh(surface,name){const E=loadEngine();Object.assign(E.cfg,{surface,drive:'rwd',auto:true,steer:'direct',uniform:true,profile:false,driven:false});E.VERT.amp=0;E.applyDriveSetup('rwd');
  const s=SETUPS[name];if(s){E.cfg.lsd=s.lsd;E.cfg.lsdCoast=s.lsdCoast;E.cfg.roll=s.roll;E.CAR.toeR=s.toeR/DEG;E.CAR.bsR=s.rs*2/E.CAR.tR;}E.selectTrack('rundkurs');return E;}
function setSpeed(E,v,gi){const S=E.S,CAR=E.CAR;S.vx=v;S.vy=0;S.r=0;S.delta=0;S.dComp=0;S.psi=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.gi=gi||3;S.rpm=w*CAR.gears[S.gi].r*CAR.final*9.55;S.we=S.rpm*0.10472;S.shiftLock=0.4;E.vertInit();}
function hold(E,vT,c){const e=vT-Math.hypot(E.S.vx,E.S.vy);c.I=clamp((c.I||0)+e*E.DT*0.6,-0.9,0.9);const u=0.35*e+c.I;E.IN.thr=clamp(u,0,1);E.IN.brk=clamp(-u*0.5,0,1);}
function catchSlide(E,vk){const S=E.S;setSpeed(E,vk/3.6);let t=0,bMax=0,rec=null;for(let i=0;i<Math.round(4/E.DT);i++){const b=Math.atan2(S.vy,Math.abs(S.vx));
  if(t<0.7){E.IN.steer=0.6;E.IN.thr=1;}else{E.IN.steer=clamp(1.2*b/E.CAR.maxSteer,-1,1);E.IN.thr=0;}E.step(E.DT);t+=E.DT;bMax=Math.max(bMax,Math.abs(b)*DEG);if(t>0.7&&rec===null&&Math.abs(b)*DEG<5&&bMax>10)rec=t-0.7;}return {bMax,rec};}
function liftOff(E,surface){const S=E.S;const vT=(surface==='tarmac'?70:60)/3.6,ayT=surface==='tarmac'?0.6:0.45;setSpeed(E,vT);const c={};for(let i=0;i<720;i++){hold(E,vT,c);E.step(E.DT);}
  E.IN.steer=((E.CAR.a+E.CAR.b)/(vT*vT)+(1.5/DEG)/G)*ayT*G/E.CAR.maxSteer;for(let i=0;i<1080;i++){hold(E,vT,c);E.step(E.DT);}const r0=S.r;let rMax=r0,bMax=0;
  for(let i=0;i<540;i++){E.IN.thr=0;E.IN.brk=0;E.step(E.DT);rMax=Math.max(rMax,S.r);bMax=Math.max(bMax,Math.abs(Math.atan2(S.vy,S.vx))*DEG);}return {dr:(rMax/r0-1)*100,bMax};}
function power(E,surface){const S=E.S;const vT=60/3.6;setSpeed(E,vT);const c={};for(let i=0;i<720;i++){hold(E,vT,c);E.step(E.DT);}
  E.IN.steer=((E.CAR.a+E.CAR.b)/(vT*vT)+(2/DEG)/G)*0.45*G/E.CAR.maxSteer;for(let i=0;i<1080;i++){hold(E,vT,c);E.step(E.DT);}const r0=S.r;let bMax=0;
  for(let i=0;i<360;i++){E.IN.thr=1;E.IN.brk=0;E.step(E.DT);bMax=Math.max(bMax,Math.abs(Math.atan2(S.vy,S.vx))*DEG);}return {dr:(S.r/r0-1)*100,bMax};}
function K(E,surface){const S=E.S,L=E.CAR.a+E.CAR.b,vT=60/3.6;setSpeed(E,vT);const c={};for(let i=0;i<720;i++){hold(E,vT,c);E.step(E.DT);}let t=0,ayMax=0;const lin=[];let past=false;
  while(t<40){const d=0.5/DEG*t;E.IN.steer=d/E.CAR.maxSteer;hold(E,vT,c);E.step(E.DT);t+=E.DT;const ay=Math.abs(S.ay);if(ay>=0.3*G)past=true;if(!past&&ay>0.1*G)lin.push([ay,d]);ayMax=Math.max(ayMax,ay);if(ay<0.9*ayMax&&t>5)break;}
  const n=lin.length;let sx=0,sy=0,sxx=0,sxy=0;for(const [a,d] of lin){sx+=a;sy+=d;sxx+=a*a;sxy+=a*d;}return {K:((n*sxy-sx*sy)/(n*sxx-sx*sx)-L/(vT*vT))*G*DEG,ay:ayMax/G};}
console.log('RWD setups compared (β in degrees, yaw-rate change in %):');
console.log('setup                       surface  | K °/g  ay,max | catch 40: β max, time | catch 70: β max, time | lift-off 0.6/0.45 g: Δr, β | throttle stab 0.45 g: Δr, β');
for(const name of Object.keys(SETUPS))for(const surface of ['tarmac','gravel']){
  const k=K(fresh(surface,name),surface),c40=catchSlide(fresh(surface,name),40),c70=catchSlide(fresh(surface,name),70),l=liftOff(fresh(surface,name),surface),p=power(fresh(surface,name),surface);
  console.log(name.padEnd(27),surface.padEnd(8),'|',f(k.K,2).padStart(5),' ',f(k.ay,2),' |',f(c40.bMax,0).padStart(4),'°',(c40.rec!==null?f(c40.rec)+' s':'never').padStart(7),'   |',f(c70.bMax,0).padStart(4),'°',(c70.rec!==null?f(c70.rec)+' s':'never').padStart(7),'   |',f(l.dr,0).padStart(5),'%',f(l.bMax,0).padStart(4),'°       |',f(p.dr,0).padStart(5),'%',f(p.bMax,0).padStart(4),'°');}
