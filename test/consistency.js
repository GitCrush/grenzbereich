// Consistency check: manoeuvres × drive × surface × settings, with plausibility rules.
const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=2)=>(x==null?'–':(+x).toFixed(d));
const issues=[],notes=[];let nTests=0;
function flag(sev,scen,msg){issues.push({sev,scen,msg});}
function fresh(cfg){const E=loadEngine();E.PED.threshold=false;Object.assign(E.cfg,{surface:'tarmac',drive:'awd',abs:false,tc:false,auto:true,steer:'direct',uniform:true,ret:false,profile:false,driven:false,hazards:'off'},cfg||{});E.VERT.amp=cfg&&cfg.rough!==undefined?cfg.rough:0;E.applyDriveSetup(E.cfg.drive);E.selectTrack(cfg&&cfg.track||'rundkurs');return E;}
function setSpeed(E,v,gi){const S=E.S,CAR=E.CAR;S.vx=v;S.vy=0;S.r=0;S.delta=0;S.dComp=0;S.psi=0;S.x=0;S.y=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.Fxr=[0,0,0,0];S.Fyr=[0,0,0,0];S.ax=0;S.ay=0;
  if(gi){S.gi=gi;}else{S.gi=2;for(let g=CAR.gears.length-1;g>=2;g--){const rpm=w*CAR.gears[g].r*CAR.final*9.55;if(rpm>=2900){S.gi=g;break;}}}
  S.rpm=Math.max(CAR.idle,w*CAR.gears[S.gi].r*CAR.final*9.55);S.we=S.rpm*0.10472;S.shiftLock=0.4;S.shiftCut=0;S.boost=0;E.vertInit();}
const v=E=>Math.hypot(E.S.vx,E.S.vy);
function hold(E,vT,c){const e=vT-v(E);c.I=clamp((c.I||0)+e*E.DT*0.6,-0.9,0.9);const u=0.35*e+c.I;E.IN.thr=clamp(u,0,1);E.IN.brk=clamp(-u*0.5,0,1);}
function sane(E,scen){const S=E.S,V=E.VERT;const vals=[S.vx,S.vy,S.r,S.psi,S.rpm,S.we,V.z,V.th,V.ph,...S.w,...S.Fz,...V.zu];
  if(vals.some(x=>!isFinite(x))){flag('ERROR',scen,'NaN/Inf in state');return false;}
  if(Math.abs(V.z)>0.35)flag('WARN',scen,'body heave '+f(V.z*1000,0)+' mm');
  if(S.Fz.some(x=>x<-1))flag('ERROR',scen,'negative wheel load');return true;}
function run(E,T,fn,scen){const n=Math.round(T/E.DT);for(let i=0;i<n;i++){fn&&fn(i*E.DT);E.step(E.DT);if(i%180===0&&!sane(E,scen))return false;}return sane(E,scen);}

const PART=+(process.env.MATRIX_PART||0);   // 1 = everything before the phantom laps, 2 = phantom, autopilot, tutor, random stages, drift, 3 = race autopilot; 0 = all
if(PART===0||PART===1){
// ---------- 1) At rest: car standing, nothing moves ----------
for(const drive of ['awd','fwd','rwd'])for(const surface of ['tarmac','gravel','snow']){
  const scen=`rest ${drive} ${surface}`;nTests++;const E=fresh({drive,surface});const S=E.S;
  run(E,4,null,scen);
  const sumFz=S.Fz.reduce((a,b)=>a+b,0);
  if(Math.abs(sumFz-E.CAR.m*G)>0.03*E.CAR.m*G)flag('WARN',scen,'sum of wheel loads '+f(sumFz,0)+' N ≠ m·g '+f(E.CAR.m*G,0));
  if(v(E)>0.15)flag('WARN',scen,'car creeps without input: '+f(v(E)*3.6,2)+' km/h');
  if(Math.abs(S.rpm-E.CAR.idle)>120)flag('WARN',scen,'idle '+f(S.rpm,0)+' instead of '+E.CAR.idle);
  if(Math.abs(S.Fz[0]-S.Fz[1])>60||Math.abs(S.Fz[2]-S.Fz[3])>60)flag('WARN',scen,'wheel loads left/right asymmetric '+S.Fz.map(x=>f(x,0)).join('/'));
}
// ---------- 2) Acceleration, all drives/surfaces, ordering ----------
const t100={};
for(const drive of ['awd','fwd','rwd'])for(const surface of ['tarmac','gravel','snow']){
  const scen=`0–100 ${drive} ${surface}`;nTests++;const E=fresh({drive,surface});const S=E.S;let t=0,t100v=null,psiMax=0;const psi0=S.psi;
  // a driver meters the throttle against wheelspin instead of holding it flat
  const dr=drive==='fwd'?[0,1]:drive==='rwd'?[2,3]:[0,1,2,3];
  while(t<25){const kMax=Math.max(...dr.map(i=>S.kappa[i]));E.IN.thr=1-0.8*clamp((kMax-0.25)/0.5,0,1);E.step(E.DT);t+=E.DT;psiMax=Math.max(psiMax,Math.abs(S.psi-psi0)*DEG);if(v(E)>=100/3.6){t100v=t;break;}}
  sane(E,scen);t100[drive+surface]=t100v;
  if(t100v===null)flag('ERROR',scen,'does not reach 100 km/h in 25 s');
  /* Window per surface (decision 7 Oct): two-wheel drive on snow is traction-bound. Friction bound with half the weight
     on the driven axle: (0.52·0.5 − rolling 0.03)·g = 2.3 m/s² → 12 s, before drag and shifts; an ideal slip
     controller holding the peak slip (κ = kP) reaches 100 km/h in 14.7 s (RWD) and 15.6 s (FWD), this metering in
     16.2 / 16.7 s, the game's keyboard model (PED.meter) in 15.7 / 16.3 s. The old common limit of 14 s flagged the
     physics, not a fault; snow now allows 18 s. */
  else if(t100v<3.2||t100v>(surface==='snow'?18:14))flag('WARN',scen,'0–100 in '+f(t100v)+' s');
  if(psiMax>2)flag('WARN',scen,'car pulls under acceleration: '+f(psiMax,1)+'° yaw');
}
for(const drive of ['awd','fwd','rwd']){const a=t100[drive+'tarmac'],b=t100[drive+'gravel'],c=t100[drive+'snow'];
  if(a&&b&&c&&!(a<b&&b<c))flag('WARN','0–100 '+drive,'ordering tarmac<gravel<snow violated: '+f(a)+'/'+f(b)+'/'+f(c));}
if(t100.awdgravel&&t100.rwdgravel&&t100.awdgravel>t100.rwdgravel)flag('WARN','0–100 gravel','rear-wheel drive faster than all-wheel drive');
// ---------- 3) Braking: ordering, ABS benefit, directional stability ----------
const brk={};
for(const surface of ['tarmac','gravel','snow'])for(const abs of [false,true]){
  const scen=`braking ${surface}${abs?'+ABS':''}`;nTests++;const E=fresh({surface,abs});const S=E.S;setSpeed(E,100/3.6);const c={};run(E,1,()=>hold(E,100/3.6,c),scen);
  const x0=S.x,y0=S.y,psi0=S.psi;E.IN.thr=0;E.IN.brk=1;let t=0;while(v(E)>0.3&&t<9){E.step(E.DT);t+=E.DT;}sane(E,scen);
  const d=Math.hypot(S.x-x0,S.y-y0);brk[surface+abs]=d;
  if(t>=9)flag('ERROR',scen,'does not come to a stop');
  if(Math.abs((S.psi-psi0)*DEG)>4)flag('WARN',scen,'car rotates under braking by '+f((S.psi-psi0)*DEG,1)+'°');
  if(Math.abs(S.y-y0)>1.5)flag('WARN',scen,'lateral drift under braking '+f(S.y-y0,2)+' m');
}
// on tarmac ABS must not lengthen the distance; on loose gravel locked wheels build a wedge and stop shorter (review M2, NHTSA: ABS +27 %)
if(brk['tarmactrue']>brk['tarmacfalse']*1.15)flag('WARN','braking tarmac','ABS lengthens the braking distance markedly: '+f(brk['tarmactrue'],1)+' vs '+f(brk['tarmacfalse'],1)+' m');
if(brk['graveltrue']<brk['gravelfalse']*1.05)flag('WARN','braking gravel','locked wheels should stop shorter than ABS on loose gravel: ABS '+f(brk['graveltrue'],1)+' vs locked '+f(brk['gravelfalse'],1)+' m');
notes.push('braking 100-0: gravel ABS '+f(brk['graveltrue'],1)+' m, locked '+f(brk['gravelfalse'],1)+' m; tarmac ABS '+f(brk['tarmactrue'],1)+' m, locked '+f(brk['tarmacfalse'],1)+' m');
if(!(brk['tarmacfalse']<brk['gravelfalse']&&brk['gravelfalse']<brk['snowfalse']))flag('WARN','braking','ordering of braking distances violated');
// ---------- 4) Skidpad ramp: left/right symmetry, ordering, understeer ----------
function ramp(E,vT,sign){const S=E.S,L=E.CAR.a+E.CAR.b;setSpeed(E,vT);const c={};run(E,2,()=>hold(E,vT,c));let t=0,ayMax=0,dAt=0,uf=0,ur=0,past=false;const lin=[];
  while(t<50){const d=sign*0.5/DEG*t;E.IN.steer=d/E.CAR.maxSteer;hold(E,vT,c);E.step(E.DT);t+=E.DT;const ay=Math.abs(S.ay);
    if(ay>=0.3*G)past=true;                       // linear range only before 0.3 g is first reached
    if(!past&&ay>0.1*G&&ay<0.3*G)lin.push([ay,Math.abs(d)]);
    if(ay>ayMax){ayMax=ay;dAt=Math.abs(d)*DEG;uf=Math.max(S.slip[0],S.slip[1]);ur=Math.max(S.slip[2],S.slip[3]);}
    if(ay<0.9*ayMax&&Math.abs(d)*DEG>dAt+2)break;}
  const n=lin.length;let sx=0,sy=0,sxx=0,sxy=0;for(const [a,d] of lin){sx+=a;sy+=d;sxx+=a*a;sxy+=a*d;}
  const K=n>10?((n*sxy-sx*sy)/(n*sxx-sx*sx)-L/(vT*vT))*G*DEG:null;
  return {ayMax:ayMax/G,dAt,K};}
const skid={};
for(const drive of ['awd','fwd','rwd'])for(const surface of ['tarmac','gravel','snow']){
  const vT=(surface==='snow'?50:60)/3.6;const scen=`Skidpad ${drive} ${surface}`;nTests++;
  const L=ramp(fresh({drive,surface}),vT,1),R=ramp(fresh({drive,surface}),vT,-1);skid[drive+surface]=L;
  if(Math.abs(L.ayMax-R.ayMax)>0.02)flag('WARN',scen,'left/right asymmetric: ay,max '+f(L.ayMax)+' / '+f(R.ayMax)+' g');
  if(Math.abs(L.dAt-R.dAt)>0.8)flag('WARN',scen,'left/right: steer angle at the peak '+f(L.dAt,1)+' / '+f(R.dAt,1)+'°');
  if(L.K!==null&&L.K<-0.5)flag('WARN',scen,'oversteer in the linear range (K = '+f(L.K)+' °/g)');
  if(L.K!==null&&L.K>8)flag('WARN',scen,'very strong understeer (K = '+f(L.K)+' °/g)');
}
for(const d of ['awd','fwd','rwd']){const a=skid[d+'tarmac'].ayMax,b=skid[d+'gravel'].ayMax,c=skid[d+'snow'].ayMax;if(!(a>b&&b>c))flag('WARN','Skidpad '+d,'ordering of ay,max violated '+f(a)+'/'+f(b)+'/'+f(c));
  notes.push('Skidpad '+d+': ay,max tarmac/gravel/snow '+f(a)+'/'+f(b)+'/'+f(c)+' g, understeer gradient '+f(skid[d+'tarmac'].K,1)+'/'+f(skid[d+'gravel'].K,1)+'/'+f(skid[d+'snow'].K,1)+' °/g');}
notes.push('0–100 km/h: '+['awd','fwd','rwd'].map(d=>d+' '+['tarmac','gravel','snow'].map(s=>f(t100[d+s],1)).join('/')).join('  ')+' s');
notes.push('braking 100–0 without/with ABS: '+['tarmac','gravel','snow'].map(s=>s+' '+f(brk[s+'false'],1)+'/'+f(brk[s+'true'],1)).join('  ')+' m');
{const a=skid.awdtarmac.ayMax,b=skid.fwdtarmac.ayMax,c=skid.rwdtarmac.ayMax;if(Math.abs(a-b)>0.08||Math.abs(a-c)>0.08)flag('WARN','Skidpad','ay,max depends strongly on the drive type: awd '+f(a)+' fwd '+f(b)+' rwd '+f(c));}
// ---------- 5) Step steer: overshoot, settling; symmetry ----------
for(const drive of ['awd','fwd','rwd'])for(const surface of ['tarmac','gravel']){
  const scen=`step steer ${drive} ${surface}`;nTests++;const res=[];
  for(const sign of [1,-1]){const E=fresh({drive,surface});const S=E.S;const vT=80/3.6;setSpeed(E,vT);const c={};run(E,2,()=>hold(E,vT,c));
    const K=skid[drive+surface].K||1.5;const ayT=surface==='tarmac'?0.4:0.35;const d=sign*((E.CAR.a+E.CAR.b)/(vT*vT)+(K/DEG)/G)*ayT*G;E.IN.steer=d/E.CAR.maxSteer;
    const rs=[];let t=0;while(t<3){hold(E,vT,c);E.step(E.DT);t+=E.DT;rs.push(Math.abs(S.r));}
    const ss=rs.slice(rs.length-180).reduce((a,b)=>a+b)/180;let rMax=0;for(const r of rs)rMax=Math.max(rMax,r);
    const tail=rs.slice(rs.length-180);const spread=(Math.max(...tail)-Math.min(...tail))/ss;
    res.push({over:(rMax/ss-1)*100,ss:ss*DEG,spread});sane(E,scen);}
  if(res[0].over>60)flag('WARN',scen,'yaw overshoot '+f(res[0].over,0)+' %');
  if(res[0].spread>0.15)flag('WARN',scen,'yaw rate does not settle (variation '+f(res[0].spread*100,0)+' %)');
  if(Math.abs(res[0].ss-res[1].ss)>0.5)flag('WARN',scen,'left/right steady-state yaw rate '+f(res[0].ss,1)+' / '+f(res[1].ss,1)+' °/s');
}
// ---------- 6) Load change in a corner: lift-off / throttle stab – direction of the reaction ----------
for(const drive of ['awd','fwd','rwd'])for(const surface of ['tarmac','gravel']){
  const scen=`load change ${drive} ${surface}`;nTests++;
  const go=(action)=>{const E=fresh({drive,surface});const S=E.S;const vT=(surface==='tarmac'?70:60)/3.6;setSpeed(E,vT);const c={};run(E,2,()=>hold(E,vT,c));
    const K=skid[drive+surface].K||1.5;const ayT=surface==='tarmac'?0.6:0.45;const d=((E.CAR.a+E.CAR.b)/(vT*vT)+(K/DEG)/G)*ayT*G;E.IN.steer=d/E.CAR.maxSteer;
    run(E,3,()=>hold(E,vT,c));const r0=S.r,b0=Math.atan2(S.vy,S.vx);let rMax=r0,rMin=r0,bMax=Math.abs(b0);
    for(let i=0;i<360;i++){if(action==='lift'){E.IN.thr=0;E.IN.brk=0;}else{E.IN.thr=1;E.IN.brk=0;}E.step(E.DT);rMax=Math.max(rMax,S.r);rMin=Math.min(rMin,S.r);bMax=Math.max(bMax,Math.abs(Math.atan2(S.vy,S.vx)));}
    sane(E,scen);return {r0:r0*DEG,rMax:rMax*DEG,rMin:rMin*DEG,bMax:bMax*DEG};};
  const L=go('lift'),P=go('power');
  if(L.rMax<L.r0*1.02&&L.rMin<L.r0*0.85)flag('INFO',scen,'lift-off reduces the yaw rate ('+f(L.r0,1)+' → min '+f(L.rMin,1)+' °/s) – no lift-off rotation');
  if(L.bMax>60)flag('WARN',scen,'lift-off at '+(surface==='tarmac'?'0.6':'0.45')+' g leads to a spin (β '+f(L.bMax,0)+'°)');
  if(drive==='fwd'&&P.rMax>P.r0*1.15)flag('INFO',scen,'throttle stab with front-wheel drive rotates the car (r '+f(P.r0,1)+' → '+f(P.rMax,1)+')');
  if(drive==='rwd'&&P.rMax<P.r0*1.05&&P.bMax<15)flag('INFO',scen,'throttle stab with rear-wheel drive without oversteer reaction');
  if(P.bMax>75)flag('INFO',scen,'throttle stab leads to a spin (β '+f(P.bMax,0)+'°) – plausible for '+drive+'?');
}
// ---------- 7) Catching a slide (counter-steer, throttle off) ----------
/* Decisions 7 Oct: (1) the slip angle is the true one, atan2(vy, vx): with |vx| a car sliding backwards after a spin
   read as "recovered". (2) The driver reacts 0.2 s after the slide passes 8° (at 0.7 s at the latest) instead of holding
   full throttle and 0.6 steer for a fixed 0.7 s: on snow and gravel a rear-wheel-drive car was already 21–27° sideways
   and rotating when the catch began, and the test measured the provocation, not the catch (RWD snow 70 km/h: no
   recovery within 4 s, β 64°; reacting at 0.2 s: β max 37°, back in 2.2 s). The late catch is kept as a note for RWD.
   Throttle stays off in the catch: 20 % throttle on snow spun the RWD car (tried). */
function catchRun(drive,surface,vk,late){const E=fresh({drive,surface});const S=E.S;setSpeed(E,vk/3.6);let t=0,bMax=0,recT=null,tC=late?0.7:null,ok=true;
  for(let i=0;i<Math.round(4/E.DT);i++){const b=Math.atan2(S.vy,S.vx);
    if(tC===null&&Math.abs(b)>8/57.3)tC=Math.min(0.7,t+0.2);const prov=tC===null?t<0.7:t<tC;
    if(prov){E.IN.steer=0.6;E.IN.thr=1;E.IN.hb=(drive==='fwd')?(t<0.4?1:0):0;}else{E.IN.hb=0;E.IN.steer=clamp(1.3*b/E.CAR.maxSteer,-1,1);E.IN.thr=0;}
    E.step(E.DT);t+=E.DT;bMax=Math.max(bMax,Math.abs(b)*DEG);if(!prov&&recT===null&&Math.abs(b)*DEG<5&&bMax>10)recT=t;}
  return {E,bMax,recT};}
for(const drive of ['awd','fwd','rwd'])for(const surface of ['tarmac','gravel','snow'])for(const vk of [40,70]){
  const scen=`catch ${drive} ${surface} ${vk}`;nTests++;const {E,bMax,recT}=catchRun(drive,surface,vk,false);
  sane(E,scen);
  if(bMax>10&&recT===null)flag('WARN',scen,'does not recover (β max '+f(bMax,0)+'°)');
  if(bMax>85)flag('INFO',scen,'β max '+f(bMax,0)+'° despite counter-steer');
  if(drive==='rwd'&&surface!=='tarmac'&&vk===70){const L=catchRun(drive,surface,vk,true);notes.push(scen+': reacting at 0.2 s β max '+f(bMax,0)+'°'+(recT?', back in '+f(recT,1)+' s':', not back')+'; late catch after 0.7 s of full throttle β max '+f(L.bMax,0)+'°'+(L.recT?', back in '+f(L.recT,1)+' s':', not back within 4 s'));}
}
// ---------- 8) Handbrake: rear only, engine keeps its revs (AWD) ----------
for(const surface of ['tarmac','gravel']){const scen=`handbrake ${surface}`;nTests++;const E=fresh({surface});const S=E.S;setSpeed(E,60/3.6);run(E,0.5);const rpm0=S.rpm;
  E.IN.hb=1;E.IN.steer=0.4;E.IN.thr=0;let wF=1,wR=1,rpmMin=1e9;for(let i=0;i<216;i++){E.step(E.DT);wF=Math.min(wF,(S.w[0]+S.w[1])/2*E.CAR.Rw/Math.max(v(E),0.1));wR=Math.min(wR,(S.w[2]+S.w[3])/2*E.CAR.Rw/Math.max(v(E),0.1));rpmMin=Math.min(rpmMin,S.rpm);}
  sane(E,scen);if(wR>0.3)flag('WARN',scen,'rear wheels do not lock (min '+f(wR)+' of vehicle speed)');if(wF<0.5)flag('WARN',scen,'front wheels lock as well');if(rpmMin<1500)flag('WARN',scen,'engine drops to '+f(rpmMin,0)+' rpm');}
// ---------- 9) Straight-line stability: fast, rough, all drives ----------
for(const drive of ['awd','fwd','rwd'])for(const [surface,rough,driven] of [['tarmac',0,false],['gravel',1,false],['gravel',1,true]]){
  const scen=`straight ${drive} ${surface}${rough?' rough':''}${driven?' line':''}`;nTests++;const E=fresh({drive,surface,rough,driven,track:'bremse'});const S=E.S;
  // long straight of the braking box, line centred; zero steering, no controller
  const T=E.TRACK;for(const q of E.LINE)q.o=0;const vT=100/3.6;setSpeed(E,vT);const p=T.pts[30];S.x=p.x;S.y=p.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=30;const c={};let rMax=0;const psi0=S.psi;
  run(E,4,()=>{E.IN.steer=0;hold(E,vT,c);rMax=Math.max(rMax,Math.abs(S.r)*DEG);},scen);
  const dpsi=Math.abs(S.psi-psi0)*DEG;
  if(rMax>2||dpsi>2)flag('WARN',scen,'yaw rate up to '+f(rMax,1)+' °/s, heading rotated by '+f(dpsi,1)+'° with zero steering');
}
// ---------- 10) Top speed and power ----------
{const scen='top speed';nTests++;const E=fresh({});const S=E.S;E.IN.thr=1;let t=0;while(t<60){E.step(E.DT);t+=E.DT;}sane(E,scen);
  const vm=v(E);const P=0.5*1.2*E.CAR.CdA*vm*vm*vm+E.SURF.tarmac.roll*E.CAR.m*G*vm;   // power at the wheels
  notes.push('v max '+f(vm*3.6,0)+' km/h in gear '+E.CAR.gears[S.gi].n+' at '+f(S.rpm,0)+' rpm, demand '+f(P/1000,0)+' kW at the wheels');
  if(vm*3.6<170||vm*3.6>230)flag('WARN',scen,'v max '+f(vm*3.6,0)+' km/h');}
// ---------- 11) Determinism ----------
{const scen='determinism';nTests++;const go=()=>{const E=fresh({surface:'gravel',rough:1});const S=E.S;E.IN.thr=1;E.IN.steer=0.2;run(E,5,null,scen);return [S.x,S.y,S.psi,S.rpm];};
  const a=go(),b=go();if(a.some((x,i)=>Math.abs(x-b[i])>1e-9))flag('WARN',scen,'two identical runs differ: '+a.map(f).join(',')+' / '+b.map(f).join(','));}
// ---------- 12) Review M1, M4–M6: sand lateral grip, wet tarmac over speed, puddles on loose ground, braking on a grade ----------
// Thresholds, decided from the measurements before → after the change (test/review.js), each with room on both sides:
//   M4 sand/gravel lateral limit ≤ 0.92 (before 0.96, after 0.89; µ ratio 0.75 plus the stronger sand plough)
//   M5 wet/dry at 110 km/h at least 0.025 below 60 km/h (before 0.75 → 0.77, after 0.74 → 0.70)
//   M6 10 mm puddle at 100 km/h: loose ground keeps ≥ 0.5 of its grip (before 0.10, after 0.70), tarmac ≤ 0.3 (0.08)
//   M1 ABS braking on 15 % downhill: 98.9 % ± 4 of the flat tyre deceleration (before 83 %, after 100 %)
{const scen='sand lateral grip (M4)';nTests++;const g=ramp(fresh({surface:'gravel'}),60/3.6,1).ayMax,sd=ramp(fresh({surface:'sand'}),60/3.6,1).ayMax;
  // the sinkage term acts along the wheel only; sideways the plough supports – sand (µ 75 % of gravel) must stay well below gravel
  if(sd/g>0.92)flag('WARN',scen,'sand reaches '+f(sd/g*100,0)+' % of the gravel lateral limit ('+f(sd)+' / '+f(g)+' g)');
  notes.push('Skidpad awd 60 km/h: sand '+f(sd)+' g against gravel '+f(g)+' g ('+f(sd/g*100,0)+' %)');}
{const scen='wet tarmac over speed (M5)';nTests++;const r=vk=>ramp(fresh({surface:'tarmac',weather:'rain'}),vk/3.6,1).ayMax/ramp(fresh({surface:'tarmac'}),vk/3.6,1).ayMax;
  const r60=r(60),r110=r(110);if(!(r110<r60-0.025))flag('WARN',scen,'wet/dry lateral limit does not fall with speed: '+f(r60)+' at 60, '+f(r110)+' at 110 km/h');
  notes.push('wet/dry lateral limit: '+f(r60)+' at 60 km/h, '+f(r110)+' at 110 km/h');}
{const scen='puddle on loose ground (M6)';nTests++;
  const grip=(surface,vk)=>{const E=fresh({surface,weather:'rain',track:'bremse'});const T=E.TRACK;T.patches=[{i0:120,i1:126,n0:-5,n1:5,type:'puddle',depth:0.010,seed:1,s1:0,s2:0,s3:0}];
    T.patchAt=new Array(T.N);for(let i=120;i<=126;i++)T.patchAt[i]=[T.patches[0]];return E.hazardAt(123,0,vk/3.6).mu;};
  const gt=grip('tarmac',100),gg=grip('gravel',100),gs=grip('sand',100);
  if(gg<0.5||gs<0.5)flag('WARN',scen,'a 10 mm puddle at 100 km/h leaves '+f(gg)+' (gravel) / '+f(gs)+' (sand) of the grip – hydroplaning on loose ground');
  if(gt>0.3)flag('WARN',scen,'tarmac no longer hydroplanes in 10 mm at 100 km/h (grip '+f(gt)+')');
  notes.push('10 mm puddle at 100 km/h, grip factor: tarmac '+f(gt)+', gravel '+f(gg)+', sand '+f(gs));}
{const scen='braking on a grade (M1)';nTests++;
  // a Gaussian hill on a long straight, braked at its inflection (15 %, no vertical curvature): the tyre deceleration
  // follows the wheel load m·g·cos θ – the longitudinal acceleration lies in the road plane and presses nothing into it
  const RAMP={name:'Ramp',type:'seg',segs:[{s:900},{R:60,a:180},{s:900},{R:60,a:180}],w:7,prof:{z:[[450,43.7,250]],bank:[]}};
  const go=(prof,s0)=>{const E=fresh({abs:true,profile:prof});E.TRACKS.ramp=RAMP;E.selectTrack('ramp');const S=E.S,T=E.TRACK;let i=0;while(T.pts[i].s<s0)i++;
    const p=T.pts[i];S.x=p.x;S.y=p.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=i;setSpeed(E,100/3.6,4);S.x=p.x;S.y=p.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=i;E.vertInit();
    let t=0,a=0,n=0;while(t<1.2){E.IN.thr=0;E.IN.brk=1;E.IN.steer=0;E.step(E.DT);t+=E.DT;if(t>0.4){a-=S.ax;n++;}}return a/n;};
  const r=go(true,608)/go(false,608);if(Math.abs(r-0.989)>0.04)flag('WARN',scen,'full braking on 15 % downhill: tyre deceleration '+f(r*100,1)+' % of the flat value (expected ≈ 98.9 %)');
  notes.push('ABS braking on 15 % downhill: tyre deceleration '+f(r*100,1)+' % of the flat value');}
}
if(PART===0||PART===2){
// ---------- 12) Profile: phantom lap, all drives ----------
// (the phantom drives its own plan – the autopilot envelope these laps used to measure first is not used by it; dropped for runtime)
for(const drive of ['awd','fwd','rwd'])for(const surface of ['tarmac','gravel']){
  const scen=`phantom lap ${drive} ${surface} profile+line`;nTests++;const E=fresh({drive,surface,profile:true,driven:true,rough:1});const S=E.S,T=E.TRACK;
  let t=0,laps=0,prevS=T.pts[S.idx].s,tLap=null,off=0,ok=true;
  while(t<200&&laps<2){E.ghostAI();E.step(E.DT);t+=E.DT;const s=T.pts[S.idx].s;if(s<prevS-800){laps++;if(laps===1)tLap=t;if(laps===2)tLap=t-tLap;}prevS=s;if(laps>=1&&!S.onTrack)off+=E.DT;if(Math.round(t/E.DT)%360===0&&!sane(E,scen)){ok=false;break;}}
  if(ok){if(laps<2)flag('WARN',scen,'phantom does not complete two laps in 200 s');else{notes.push(scen+': '+f(tLap,1)+' s, off track '+f(off,1)+' s');if(off>4)flag('WARN',scen,'phantom '+f(off,1)+' s off track');}}
}
// ---------- 13) Gear changes: no rpm jump on upshift, downshift with blip ----------
{const scen='gear changes';nTests++;const E=fresh({});const S=E.S;E.IN.thr=1;let t=0,prevG=S.gi,bad=0,prevRpm=S.rpm;
  while(t<12){E.step(E.DT);t+=E.DT;if(S.gi>prevG&&S.rpm>prevRpm+50)bad++;prevG=S.gi;prevRpm=S.rpm;}
  if(bad>0)flag('WARN',scen,'rpm rises on upshift ('+bad+' steps)');
  // downshifts under braking: does the rpm stay below the limiter?
  E.IN.thr=0;E.IN.brk=1;let rpmMax=0;t=0;while(v(E)>2&&t<8){E.step(E.DT);t+=E.DT;rpmMax=Math.max(rpmMax,S.rpm);}
  if(rpmMax>E.CAR.limit+100)flag('WARN',scen,'over-rev on downshift: '+f(rpmMax,0)+' rpm');
}
// ---------- 14) Wheel load in cornering: inside rear at the tarmac limit ----------
for(const drive of ['awd','rwd']){const scen=`wheel load at limit ${drive} tarmac`;nTests++;const E=fresh({drive});const S=E.S;const vT=60/3.6;setSpeed(E,vT);const c={};run(E,2,()=>hold(E,vT,c));let t=0,minIn=1e9;
  while(t<25){const d=0.5/DEG*t;E.IN.steer=d/E.CAR.maxSteer;hold(E,vT,c);E.step(E.DT);t+=E.DT;if(S.ay>1.0*G)minIn=Math.min(minIn,S.Fz[0],S.Fz[2]);}
  if(minIn<50)flag('INFO',scen,'inside wheel lifts at the limit (load '+f(minIn,0)+' N) – roll distribution '+Math.round(E.cfg.roll*100)+' % front');}


// ---------- 15) Physical invariants ----------
{ // power: acceleration at 100 km/h in 3rd gear must not exceed P/(m·v)
  const scen='power balance';nTests++;const E=fresh({auto:false});const S=E.S;setSpeed(E,100/3.6,4);const c={};run(E,1,()=>hold(E,100/3.6,c));
  E.IN.thr=1;E.IN.brk=0;run(E,1.2);const ax=S.ax;const Tw=E.CAR.gears[S.gi].r*E.CAR.final*0.9;
  const Pmax=216e3;const aLim=Pmax/(E.CAR.m*v(E))+0.05;   // 290 hp at the wheels, plus reserve
  notes.push('acceleration at '+f(v(E)*3.6,0)+' km/h full throttle: '+f(ax/G)+' g, power limit '+f(aLim/G)+' g');
  if(ax>aLim)flag('WARN',scen,'acceleration '+f(ax/G)+' g above the power limit '+f(aLim/G)+' g');
}
{ // steady-state corner: ay = v·r, tyre force inside the friction circle
  const scen='steady-state cornering invariants';nTests++;const E=fresh({});const S=E.S;const vT=60/3.6;setSpeed(E,vT);const c={};run(E,2,()=>hold(E,vT,c));E.IN.steer=(5/DEG)/E.CAR.maxSteer;run(E,4,()=>hold(E,vT,c));
  const ay=S.ay,vr=v(E)*S.r;if(Math.abs(ay-vr)>0.4)flag('WARN',scen,'ay '+f(ay)+' ≠ v·r '+f(vr)+' in steady state');
  for(let i=0;i<4;i++){const Fm=Math.hypot(S.Fxr[i],S.Fyr[i]),lim=1.52*1.4*S.Fz[i]*1.05;if(Fm>lim)flag('WARN',scen,'tyre force wheel '+i+' beyond the friction circle: '+f(Fm,0)+' > '+f(lim,0)+' N');}
  if(Math.sign(S.Tw||0)===Math.sign(S.delta)&&Math.abs(S.Tw)>0.5)flag('WARN',scen,'steering torque does not self-centre (same sign as steer angle)');
  if(Math.abs(S.Tw||0)>40)flag('WARN',scen,'steering torque '+f(S.Tw,1)+' Nm');
}
{ // deceleration must not exceed µ·g
  for(const surface of ['tarmac','gravel','snow']){const scen='deceleration '+surface;nTests++;const E=fresh({surface,abs:true});const S=E.S;setSpeed(E,100/3.6);const c={};run(E,1,()=>hold(E,100/3.6,c));E.IN.thr=0;E.IN.brk=1;let axMin=0,t=0;while(v(E)>5&&t<8){E.step(E.DT);t+=E.DT;if(v(E)>8)axMin=Math.min(axMin,S.ax);}
    const mu=E.SURF[surface].mu*1.21;if(-axMin>mu*G*1.05+(surface==='tarmac'?0:0.1*G))flag('WARN',scen,'peak deceleration '+f(-axMin/G)+' g above µ·g '+f(mu));}
}
{ // rolling at constant speed: small slip with the right sign
  const scen='constant speed';nTests++;const E=fresh({});const S=E.S;const vT=80/3.6;setSpeed(E,vT);const c={};run(E,4,()=>hold(E,vT,c));
  const kd=(S.kappa[0]+S.kappa[1]+S.kappa[2]+S.kappa[3])/4;if(kd<0||kd>0.04)flag('WARN',scen,'slip at constant speed '+f(kd,3));
  if(Math.abs(S.rpm-v(E)/E.CAR.Rw*E.CAR.gears[S.gi].r*E.CAR.final*9.55)>150)flag('WARN',scen,'rpm does not match wheel speed and gear (clutch slipping?)');
}
// ---------- 16) The phantom does not disturb the player (state separation) ----------
{ const scen='phantom state separation';nTests++;
  const go=(withGhost)=>{const E=fresh({surface:'gravel',rough:1});const S=E.S;if(withGhost)E.spawnGhost();E.IN.thr=1;E.IN.steer=0.15;
    for(let i=0;i<360*5;i++){E.step(E.DT);if(withGhost)E.stepGhost&&E.stepGhost(E.DT);}return [S.x,S.y,S.psi,S.rpm];};
  const a=go(false),b=go(true);if(a.some((x,i)=>Math.abs(x-b[i])>1e-6))flag('ERROR',scen,'phantom alters the player state: '+a.map(x=>f(x,3)).join(',')+' / '+b.map(x=>f(x,3)).join(','));
}
// ---------- 17) Reversing ----------
{ const scen='reverse';nTests++;const E=fresh({auto:false});const S=E.S;S.gi=0;E.IN.thr=0.5;run(E,4,null,scen);
  const vx=S.vx;if(vx>-1)flag('WARN',scen,'reverse gear does not move the car backwards (v = '+f(vx*3.6,1)+' km/h)');
  if(vx<-40/3.6)flag('INFO',scen,'reverse '+f(-vx*3.6,0)+' km/h after 4 s at half throttle');
  E.IN.thr=0;E.IN.brk=1;run(E,3,null,scen);if(Math.abs(S.vx)>0.5)flag('WARN',scen,'braking from reverse does not stop the car');
}
// ---------- 18) Reset and back on track ----------
{ const scen='reset';nTests++;const E=fresh({});const S=E.S;E.IN.thr=1;E.IN.steer=0.3;run(E,3);E.IN.thr=0;E.IN.steer=0;E.reset();
  if(v(E)>0.01||Math.abs(S.r)>0.01||S.gi!==2)flag('WARN',scen,'state after reset not at rest: v '+f(v(E)*3.6,1)+' km/h, gear '+E.CAR.gears[S.gi].n);
  run(E,1,null,scen);if(v(E)>0.2)flag('WARN',scen,'car moves after reset without input');
}


// ---------- 19) Braking with steering (keyboard, raw and threshold): no spin ----------
for(const surface of ['tarmac','gravel','snow'])for(const thr of [false,true])for(const st of [0.15,0.3]){
  const scen=`braking+steering ${surface} ${st} ${thr?'threshold':'raw'}`;nTests++;const E=fresh({surface});E.PED.threshold=thr;const S=E.S;setSpeed(E,(surface==='snow'?80:100)/3.6);
  let t=0,bMax=0;for(let i=0;i<Math.round(4/E.DT);i++){E.keys['arrowdown']=true;if(i%6===0)E.readInput(1/60);E.IN.steer=t>0.8?st:0;E.step(E.DT);t+=E.DT;if(v(E)<3)break;bMax=Math.max(bMax,Math.abs(Math.atan2(S.vy,Math.abs(S.vx))*DEG));}
  sane(E,scen);if(bMax>30)flag('WARN',scen,'car rotates under braking with steering (β '+f(bMax,0)+'°)');
}


// ---------- 20) Automatic gearbox holds the gear in a corner ----------
for(const drive of ['rwd','awd']){
  const scen=`automatic in corner ${drive} tarmac`;nTests++;const E=fresh({drive});const S=E.S;setSpeed(E,50/3.6,2);let t=0,shifted=false,ayMax=0;
  // skidpad-like: gentle speed build-up at ~0.8 g; the automatic must shift and the car must not spin from it
  let vT=58/3.6,c={I:0},bAfter=0,tShift=null;const L=E.CAR.a+E.CAR.b;
  E.selectTrack('kreis');const T=E.TRACK;const p=T.pts[10];S.x=p.x;S.y=p.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=10;S.vx=vT;{const w=vT/E.CAR.Rw;S.w=[w,w,w,w];S.gi=2;S.rpm=w*E.CAR.gears[2].r*E.CAR.final*9.55;S.we=S.rpm*0.10472;E.vertInit();}
  for(let i=0;i<Math.round(16/E.DT);i++){let j=S.idx,acc=0;while(acc<10){acc+=E.TRACK.pts[j].seg;j=(j+1)%E.TRACK.N;}const q=E.TRACK.pts[j];const al=Math.atan2(q.y-S.y,q.x-S.x)-S.psi;const a=Math.atan2(Math.sin(al),Math.cos(al));
    E.IN.steer=clamp(Math.atan(2*L*Math.sin(a)/10)/E.CAR.maxSteer,-1,1);if(t>2)vT+=E.DT*(1.0/3.6);const vv=v(E);const e=vT-vv;c.I=clamp(c.I+e*E.DT*0.5,-0.5,0.9);E.IN.thr=clamp(0.5*e+c.I,0,1);E.IN.brk=0;
    const g0=S.gi;E.step(E.DT);t+=E.DT;if(S.gi!==g0&&tShift===null)tShift=t;if(tShift!==null&&t<tShift+1.5)bAfter=Math.max(bAfter,Math.abs(Math.atan2(S.vy,Math.abs(S.vx))*DEG));}
  sane(E,scen);if(tShift===null)flag('WARN',scen,'automatic never upshifts on the skidpad');else if(bAfter>20)flag('WARN',scen,'car spins after the automatic upshift (β '+f(bAfter,0)+'°)');
}


// ---------- 21) Tutor: silent in steady cornering, catches throttle stabs (same set-up as test/tutortest.js) ----------
for(const [drive,surface,R,vk,stab] of [['awd','gravel',40,62,0],['awd','gravel',34,54,0],['rwd','tarmac',40,68,0],['rwd','gravel',40,52,1.0],['rwd','tarmac',40,68,1.0],['awd','gravel',34,54,1.5]]){
  const scen=`tutor ${drive} ${surface} R${R} ${stab?'stab '+stab+' s':'steady'}`;nTests++;const E=fresh({drive,surface,auto:false,tutor:true,rough:1});const S=E.S,CAR=E.CAR;E.selectTrack('kreis');const T=E.TRACK;
  const p=T.pts[10],q=E.LINE[10];S.x=q.x;S.y=q.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=10;const v0=34/3.6;S.vx=v0;{const w=v0/CAR.Rw;S.w=[w,w,w,w];S.gi=3;S.rpm=w*CAR.gears[3].r*CAR.final*9.55;S.we=S.rpm*0.10472;E.vertInit();}
  let cxp=0,cyp=0;for(const pp of T.pts){cxp+=pp.x;cyp+=pp.y;}cxp/=T.N;cyp/=T.N;
  const circ=()=>{const a0=Math.atan2(S.y-cyp,S.x-cxp)+12/R;const tx=cxp+R*Math.cos(a0),ty=cyp+R*Math.sin(a0);const al=Math.atan2(ty-S.y,tx-S.x)-S.psi;const a=Math.atan2(Math.sin(al),Math.cos(al));return Math.atan(2*(CAR.a+CAR.b)*Math.sin(a)/12);};
  let t=0,sw=0,c={I:0},onT=0,bMax=0,spun=false,held=0,lastTut=null;
  for(let i=0;i<Math.round(18/E.DT);i++){const vT=(34+(vk-34)*clamp((t-2)/5,0,1))/3.6;const e=vT-v(E);c.I=clamp(c.I+e*E.DT*0.5,-0.5,0.6);let thr=clamp(0.5*e+c.I,0,0.5);if(stab&&t>10&&t<10+stab)thr=1;
    if(i%6===0)lastTut=E.tutorAdvice();const tut=lastTut;E.IN.thr=thr;E.IN.brk=0;
    let target=circ();if(t<=10)held=target;else if(stab)target=held;if(tut&&tut.on){target=tut.target;if(t>8)onT+=E.DT;}
    const rate=600/DEG/CAR.ratio*E.DT;sw+=clamp(target-sw,-rate,rate);E.IN.steer=clamp(sw/CAR.maxSteer,-1,1);E.step(E.DT);t+=E.DT;
    const b=Math.abs(Math.atan2(S.vy,Math.abs(S.vx))*DEG);if(t>9)bMax=Math.max(bMax,b);if(b>75&&t>3){spun=true;break;}}
  sane(E,scen);
  if(!stab&&onT>4)flag('WARN',scen,'tutor shows a correction in steady cornering for '+f(onT,1)+' s');
  if(stab&&spun)flag('WARN',scen,'car spins although the driver follows the tutor');
  if(stab&&!spun)notes.push(scen+': held, beta max '+f(bMax,0)+' deg, tutor on '+f(onT,1)+' s');
}


// ---------- 22) Random stages: generated, driveable, the phantom gets round ----------
for(const seed of [7,31,1234]){const scen=`random stage seed ${seed}`;nTests++;const E=fresh({surface:'gravel',profile:true,driven:true});
  const d=E.randomTrackDef(seed);if(!d){flag('ERROR',scen,'generator gave up');continue;}E.TRACKS.random=d;E.selectTrack('random');const S=E.S,T=E.TRACK;
  E.apEnsure();while(E.AP.job)E.apRun(1e9);E.apEnsure();
  let minR=1e9;for(const p of T.pts)minR=Math.min(minR,p.R);if(minR<12)flag('WARN',scen,'radius '+f(minR,0)+' m');
  let t=0,prevS=T.pts[S.idx].s,done=false,off=0;while(t<200&&!done){E.ghostAI();E.step(E.DT);t+=E.DT;const s=T.pts[S.idx].s;if(s<prevS-T.pts[T.N-1].s/2)done=true;prevS=s;if(!S.onTrack)off+=E.DT;}
  sane(E,scen);if(!done)flag('WARN',scen,'phantom did not complete a lap');else notes.push(scen+': '+d.desc+', phantom lap '+f(t)+' s, off track '+f(off)+' s');}


// ---------- 23) Autopilot drift on loose surfaces: the slip angle follows the plan ----------
// Drives the autopilot as the browser does (ghostCommand(…, 'catch'), the drift layer) and measures against its live
// target AP.brNow. Until 7 Oct this scenario drove the phantom (ghostAI, no drift) against the old layer's target – its
// figures were the phantom's. The 5° tolerance is kept: the WARNs stand until the layer holds the angle (TODO).
// Since 8 Oct it drives with the measured envelope and the racing line, as the user gets it once the plan is ready
// (before, the fallback plan of the first ~30 s in the browser), the pedal kept by the autopilot (AP.pedBrk) and the
// inputs reset every step as readInput does. With the fallback plan the angles were RWD gravel 1.8°, AWD 1.0°.
for(const [drive,surface] of [['rwd','gravel'],['awd','gravel'],['awd','sand'],['rwd','snow']]){const scen=`autopilot drift ${drive} ${surface}`;nTests++;
  const E=fresh({drive,surface,profile:true,driven:true,rough:1});E.AP.drift=0.6;E.AP.on=true;const S=E.S,T=E.TRACK,L=T.pts[T.N-1].s;   // the autopilot (Y): race plan + catch + slide layer
  E.apEnsure();while(E.AP.job)E.apRun(1e9);E.apEnsure();while(E.AP.lineJob)E.apLineRun(1e9);E.apEnsure();E.AP.lineOn=true;E.AP.profKey='';E.apEnsure();
  let t=0,laps=0,prevS=T.pts[S.idx].s,tLap=null,off=0,bS=0,rS=0,n=0;
  while(t<420&&laps<3){E.IN.steer=0;E.IN.thr=0;E.IN.brk=0;E.IN.hb=0;const c=E.ghostCommand(E.AP.pedBrk||0,E.DT,'catch');E.AP.pedBrk=c.brk;E.IN.steer=c.steer;E.IN.thr=c.thr;E.IN.brk=c.brk;E.IN.hb=c.hb||0;E.step(E.DT);t+=E.DT;const s=T.pts[S.idx].s;if(s<prevS-L/2){laps++;if(laps===2)tLap=t;if(laps===3)tLap=t-tLap;}prevS=s;
    if(laps>=2){if(!S.onTrack)off+=E.DT;const br=Math.abs(E.AP.brNow||0);if(br>5/57.3){bS+=Math.abs(Math.atan2(S.vy,Math.abs(S.vx)));rS+=br;n++;}}}
  sane(E,scen);if(laps<3){flag('WARN',scen,'did not complete three laps');continue;}
  const b=bS/Math.max(1,n)*DEG,r=rS/Math.max(1,n)*DEG;if(Math.abs(b-r)>5)flag('WARN',scen,'slip angle '+f(b)+' deg against a target of '+f(r));
  notes.push(scen+': 3rd lap '+f(tLap)+' s, slip in corners '+f(b)+' deg (target '+f(r)+'), off track '+f(off)+' s');}

}
// ---------- 24) Race autopilot (Y): measured envelope, racing line, speed profile, catch, learning – at the default pace ----------
// Part 3 of the matrix (MATRIX_PART=3), so parts 1 and 2 stay under five minutes; CI runs all parts. Rule decided 7 Oct
// from the first measurements (94 %: all of circuit/stage × tarmac/gravel × AWD/RWD/FWD on the road; 100 %: stage tarmac
// AWD 0.4 s off): more than 1 s off track per lap at the default pace is a WARN; lap times are notes to compare.
if(PART===0||PART===3){
for(const [tr,surface,drive] of [['rundkurs','tarmac','awd'],['rundkurs','gravel','awd'],['etappe','tarmac','rwd'],['etappe','gravel','fwd']]){
  const scen=`race autopilot ${tr} ${surface} ${drive}`;nTests++;const E=fresh({drive,surface,profile:true,driven:true,rough:1,track:tr});E.AP.on=true;E.AP.drift=0;
  E.apEnsure();while(E.AP.job)E.apRun(1e9);E.apEnsure();while(E.AP.lineJob)E.apLineRun(1e9);E.apEnsure();if(!process.env.NOPRE){while(E.AP.preJob)E.apPreRun(1e9);E.apEnsure();}   // envelope, then the racing line
  const S=E.S,T=E.TRACK,L=T.pts[T.N-1].s;let t=0,laps=0,prevS=T.pts[S.idx].s,tLap=null,off=0;
  while(t<200&&laps<2){if(Math.round(t/E.DT)%90===0)E.apEnsure();const c=E.ghostCommand(E.IN.brk,E.DT,'catch');E.IN.steer=c.steer;E.IN.thr=c.thr;E.IN.brk=c.brk;E.IN.hb=c.hb||0;E.step(E.DT);t+=E.DT;const s=T.pts[S.idx].s;if(s<prevS-L/2){laps++;if(laps===1)tLap=t;if(laps===2)tLap=t-tLap;}prevS=s;if(laps>=1&&!S.onTrack)off+=E.DT;}
  sane(E,scen);if(laps<2){flag('WARN',scen,'does not complete two laps');continue;}
  if(off>1)flag('WARN',scen,f(off,1)+' s off track per lap at pace '+f(E.AP.pace*100,0)+' %');
  notes.push(scen+': '+f(tLap,1)+' s, off track '+f(off,1)+' s (pace '+f(E.AP.pace*100,0)+' % of the measured limit)');}
}
// ---------- Output ----------
console.log('Consistency check:',nTests,'scenarios,',issues.length,'findings');
const order={ERROR:0,WARN:1,INFO:2};issues.sort((a,b)=>order[a.sev]-order[b.sev]);
for(const i of issues)console.log(' ',i.sev.padEnd(6),i.scen.padEnd(36),i.msg);
console.log('Notes:');for(const n of notes)console.log('  '+n);
