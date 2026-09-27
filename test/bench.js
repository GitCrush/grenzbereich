// Standard manoeuvres against the physics – headless.
const {loadEngine}=require('./load');
const DEG=180/Math.PI, G=9.81;
const clamp=(v,a,b)=>v<a?a:v>b?b:v;

function fresh(surface,opts={}){
  const E=loadEngine();
  Object.assign(E.cfg,{surface,drive:opts.drive||'awd',abs:!!opts.abs,tc:!!opts.tc,auto:opts.auto!==false,
                       steer:'direct',uniform:true,ret:false,profile:false,driven:false});
  E.VERT.amp=opts.rough===undefined?0:opts.rough;
  E.selectTrack('rundkurs');
  return E;
}
function setSpeed(E,v){
  const S=E.S,CAR=E.CAR;
  S.vx=v;S.vy=0;S.r=0;S.delta=0;S.dComp=0;
  const w=v/CAR.Rw;S.w=[w,w,w,w];S.Fxr=[0,0,0,0];S.Fyr=[0,0,0,0];S.ax=0;S.ay=0;
  S.gi=2;for(let g=CAR.gears.length-1;g>=2;g--){const rpm=w*CAR.gears[g].r*CAR.final*60/(2*Math.PI);if(rpm>=2900){S.gi=g;break;}}
  S.rpm=Math.max(CAR.idle,w*CAR.gears[S.gi].r*CAR.final*60/(2*Math.PI));S.we=S.rpm*0.10472;
  S.shiftLock=0.4;S.shiftCut=0;S.boost=0;S.psi=0;S.x=0;S.y=0;
  E.vertInit();
}
const v=E=>Math.hypot(E.S.vx,E.S.vy);
function holdSpeed(E,vT,c){ // simple PI controller on the throttle
  const e=vT-v(E); c.I=clamp((c.I||0)+e*E.DT*0.6,-0.6,0.9);
  const u=0.35*e+c.I; E.IN.thr=clamp(u,0,1); E.IN.brk=clamp(-u*0.5,0,1);
}
function steerFor(E,delta){E.IN.steer=clamp(delta/E.CAR.maxSteer,-1,1);}
function run(E,T,fn){const n=Math.round(T/E.DT);for(let i=0;i<n;i++){if(fn)fn(i*E.DT);E.step(E.DT);}}

// ---------- Acceleration 0–100 ----------
function launch(surface,opts){
  const E=fresh(surface,opts);const S=E.S;
  let t=0,t50=null,t100=null,kR=0,kF=0,d=0,rpmMin=1e9,vPrev=0,axMax=0;
  E.IN.steer=0;E.IN.thr=1;E.IN.brk=0;
  while(t<15){E.step(E.DT);t+=E.DT;const vv=v(E);d+=vv*E.DT;
    kR=Math.max(kR,S.kappa[2],S.kappa[3]);kF=Math.max(kF,S.kappa[0],S.kappa[1]);axMax=Math.max(axMax,S.ax);
    if(vv>1&&S.gi>=2)rpmMin=Math.min(rpmMin,S.rpm);
    if(t50===null&&vv>=50/3.6)t50=t;if(t100===null&&vv>=100/3.6){t100=t;break;}}
  return {t50,t100,d:d,kF,kR,axMax:axMax/G};
}
// ---------- Braking 100–0 ----------
function braking(surface,opts){
  const E=fresh(surface,opts);const S=E.S;
  setSpeed(E,100/3.6);const c={};run(E,1.0,()=>holdSpeed(E,100/3.6,c));
  const x0=S.x,y0=S.y,psi0=S.psi;E.IN.thr=0;E.IN.brk=1;E.IN.clutch=false;
  let t=0,kF=0,kR=0,tF=null,tR=null,axMin=0,yawMax=0;
  while(v(E)>0.3&&t<8){E.step(E.DT);t+=E.DT;
    const f=Math.min(S.kappa[0],S.kappa[1]),r=Math.min(S.kappa[2],S.kappa[3]);
    kF=Math.min(kF,f);kR=Math.min(kR,r);
    if(tF===null&&f<-0.9)tF=t;if(tR===null&&r<-0.9)tR=t;
    if(v(E)>3)axMin=Math.min(axMin,S.ax);yawMax=Math.max(yawMax,Math.abs(S.r)*DEG);}
  const dist=Math.hypot(S.x-x0,S.y-y0);
  return {dist,t,gMean:(100/3.6)**2/(2*dist)/G,gPeak:-axMin/G,kF,kR,tF,tR,dpsi:(S.psi-psi0)*DEG,yawMax};
}
// ---------- Steer ramp at constant speed ----------
function rampSteer(surface,vT,opts){
  const E=fresh(surface,opts);const S=E.S,L=E.CAR.a+E.CAR.b;
  setSpeed(E,vT);const c={};run(E,2.0,()=>holdSpeed(E,vT,c));
  const rate=0.5/DEG; // rad/s at the road wheel
  let t=0,delta=0;const smp=[];let ayMax=0,best=null;
  while(delta<22/DEG&&t<60){
    delta=rate*t;steerFor(E,delta);holdSpeed(E,vT,c);E.step(E.DT);t+=E.DT;
    const ay=S.ay,uf=Math.max(S.slip[0],S.slip[1]),ur=Math.max(S.slip[2],S.slip[3]);
    const rec={t,delta,ay,r:S.r,beta:Math.atan2(S.vy,S.vx),uf,ur,aF:(S.alpha[0]+S.alpha[1])/2,aR:(S.alpha[2]+S.alpha[3])/2,roll:E.VERT.ph,v:v(E),dEff:S.delta+S.dComp};
    smp.push(rec);
    if(ay>ayMax){ayMax=ay;best=rec;}
    if(best&&ay<0.90*ayMax&&delta>best.delta+2/DEG)break;
  }
  // understeer gradient from the linear range 0.10–0.30 g (intercept not forced)
  const lin=smp.filter(s=>s.ay>0.10*G&&s.ay<0.30*G);
  let K=null,rollGrad=null;
  if(lin.length>10){const n=lin.length;let sx=0,sy=0,sxx=0,sxy=0;for(const s of lin){sx+=s.ay;sy+=s.delta;sxx+=s.ay*s.ay;sxy+=s.ay*s.delta;}
    const slope=(n*sxy-sx*sy)/(n*sxx-sx*sx);K=(slope-L/(vT*vT))*G*DEG;
    let rx=0,ry=0,rxx=0,rxy=0;for(const s of lin){rx+=s.ay;ry+=s.roll;rxx+=s.ay*s.ay;rxy+=s.ay*s.roll;}
    rollGrad=((n*rxy-rx*ry)/(n*rxx-rx*rx))*G*DEG;}
  const last=smp[smp.length-1];
  return {ayMax:ayMax/G,deltaAt:best.delta*DEG,dEffAt:best.dEffAt,betaAt:best.beta*DEG,ufAt:best.uf,urAt:best.ur,aFAt:best.aF*DEG,aRAt:best.aR*DEG,K,rollGrad,
          ayEnd:last.ay/G,deltaEnd:last.delta*DEG,ufEnd:last.uf,urEnd:last.ur,vAt:best.v*3.6};
}
// ---------- Step steer ----------
function stepSteer(surface,vT,ayTarget,K,opts){
  const E=fresh(surface,opts);const S=E.S,L=E.CAR.a+E.CAR.b;
  setSpeed(E,vT);const c={};run(E,2.0,()=>holdSpeed(E,vT,c));
  const delta=(L/(vT*vT)+(K/DEG)/G)*ayTarget*G;
  steerFor(E,delta);
  const rs=[],ays=[];let t=0;
  while(t<3){holdSpeed(E,vT,c);E.step(E.DT);t+=E.DT;rs.push(S.r);ays.push(S.ay);}
  const n=rs.length,ss=rs.slice(n-Math.round(0.5/E.DT)).reduce((a,b)=>a+b,0)/Math.round(0.5/E.DT);
  const ayss=ays.slice(n-Math.round(0.5/E.DT)).reduce((a,b)=>a+b,0)/Math.round(0.5/E.DT);
  let t90=null,t90ay=null,rMax=-1e9;
  for(let i=0;i<n;i++){if(t90===null&&rs[i]>=0.9*ss)t90=i*E.DT;if(t90ay===null&&ays[i]>=0.9*ayss)t90ay=i*E.DT;rMax=Math.max(rMax,rs[i]);}
  return {delta:delta*DEG,rss:ss*DEG,ayss:ayss/G,t90,t90ay,over:(rMax-ss)/ss*100,gain:(ss/delta)};
}
// ---------- Wheel dynamics at low speed, neutral ----------
function lowSpeedWheel(surface,brk){
  const E=fresh(surface,{auto:false});const S=E.S;
  setSpeed(E,20/3.6);S.gi=1;E.IN.thr=0;E.IN.brk=0;
  run(E,0.3);
  E.IN.brk=brk;let t=0,maxJump=0,flips=0,prevDw=0,wMin=1e9,neg=0;const tr=[];
  while(t<1.5&&v(E)>0.5){const w0=S.w[0];E.step(E.DT);t+=E.DT;const dw=S.w[0]-w0;
    maxJump=Math.max(maxJump,Math.abs(dw));if(prevDw*dw<0&&Math.abs(dw)>0.05)flips++;prevDw=dw;wMin=Math.min(wMin,S.w[0]);if(S.w[0]<-0.01)neg++;
    if(tr.length<40&&(Math.round(t/E.DT)%9===0))tr.push((S.w[0]*E.CAR.Rw*3.6).toFixed(1));}
  return {maxJump,flips,wMin,neg,tr:tr.join(' ')};
}

const f=(x,d=2)=>x==null?'   –':(+x).toFixed(d);
console.log('=== Acceleration 0–100 km/h (AWD, automatic, smooth road) ===');
console.log('surface      t50    t100   dist  κF,max κR,max ax,max');
for(const s of ['tarmac','gravel','snow']){const r=launch(s);console.log(s.padEnd(11),f(r.t50),'s',f(r.t100),'s',f(r.d,0).padStart(4),'m',f(r.kF),' ',f(r.kR),' ',f(r.axMax),'g');}
{const r=launch('gravel',{rough:1});console.log('gravel+rough',f(r.t50),'s',f(r.t100),'s',f(r.d,0).padStart(4),'m',f(r.kF),' ',f(r.kR),' ',f(r.axMax),'g');}
{const r=launch('tarmac',{tc:true});console.log('tarmac+TC  ',f(r.t50),'s',f(r.t100),'s',f(r.d,0).padStart(4),'m',f(r.kF),' ',f(r.kR),' ',f(r.axMax),'g');}

console.log('\n=== Braking 100–0 km/h, full brake, bias 76 % front ===');
console.log('surface         dist   time  g,mean   g,peak   κF,min κR,min  tLockF  tLockR  Δψ    r,max');
for(const [s,o] of [['tarmac',{}],['tarmac',{abs:true}],['gravel',{}],['gravel',{abs:true}],['gravel',{rough:1}],['snow',{}],['snow',{abs:true}]]){
  const r=braking(s,o);console.log((s+(o.abs?'+ABS':'')+(o.rough?'+rough':'')).padEnd(14),f(r.dist,1).padStart(6),'m',f(r.t),'s ',f(r.gMean),'  ',f(r.gPeak),'   ',f(r.kF),' ',f(r.kR),'  ',f(r.tF),'  ',f(r.tR),'  ',f(r.dpsi,1),'°',f(r.yawMax,1),'°/s');}

console.log('\n=== Steer ramp 0.5°/s at the road wheel, constant speed ===');
console.log('surface     v      ay,max  δ@max  β@max  αF@max αR@max useF useR | K [°/g] roll[°/g] | end: δ  ay  useF useR');
const KS={};
for(const [s,vk] of [['tarmac',60],['tarmac',100],['gravel',60],['snow',50]]){
  const r=rampSteer(s,vk/3.6);KS[s]=r.K;
  console.log(s.padEnd(10),String(vk).padStart(3),'  ',f(r.ayMax),' ',f(r.deltaAt,1).padStart(5),' ',f(r.betaAt,1).padStart(5),' ',f(r.aFAt,1).padStart(5),' ',f(r.aRAt,1).padStart(5),' ',f(r.ufAt),f(r.urAt),'|',f(r.K),' ',f(r.rollGrad),'   |',f(r.deltaEnd,1),f(r.ayEnd),f(r.ufEnd),f(r.urEnd));
}
console.log('\n=== Step steer at 80 km/h to 0.4 g (snow 0.3 g) ===');
console.log('surface     δ     r,ss    ay,ss  t90(r)  t90(ay) overshoot  yaw gain[1/s]');
for(const [s,ay] of [['tarmac',0.4],['gravel',0.4],['snow',0.3]]){
  const r=stepSteer(s,80/3.6,ay,KS[s]||0);
  console.log(s.padEnd(10),f(r.delta,2),' ',f(r.rss,1).padStart(5),' ',f(r.ayss),'  ',f(r.t90,3),' ',f(r.t90ay,3),'  ',f(r.over,1).padStart(5),'%   ',f(r.gain));
}
console.log('\n=== Wheel dynamics 20 km/h, neutral, brake 30 % / 100 % (wheel FL, rim speed km/h every 25 ms) ===');
for(const b of [0.3,1.0]){const r=lowSpeedWheel('tarmac',b);console.log('brk',b,' maxΔω/step',f(r.maxJump,3),'rad/s  sign changes',r.flips,' ω,min',f(r.wMin,2),' negative',r.neg);console.log('   ',r.tr);}
