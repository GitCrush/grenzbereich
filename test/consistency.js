// Consistency check: manoeuvres × drive × surface × settings, with plausibility rules.
const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=2)=>(x==null?'–':(+x).toFixed(d));
const issues=[],notes=[];let nTests=0;
function flag(sev,scen,msg){issues.push({sev,scen,msg});}
function fresh(cfg){const E=loadEngine();E.PED.threshold=false;Object.assign(E.cfg,{surface:'tarmac',drive:'awd',abs:false,tc:false,auto:true,steer:'direct',uniform:true,ret:false,profile:false,driven:false},cfg||{});E.VERT.amp=cfg&&cfg.rough!==undefined?cfg.rough:0;E.applyDriveSetup(E.cfg.drive);E.selectTrack(cfg&&cfg.track||'rundkurs');return E;}
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
  else if(t100v<3.2||t100v>14)flag('WARN',scen,'0–100 in '+f(t100v)+' s');
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
for(const s of ['tarmac','gravel','snow'])if(brk[s+'true']>brk[s+'false']*1.15)flag('WARN','braking '+s,'ABS lengthens the braking distance markedly: '+f(brk[s+'true'],1)+' vs '+f(brk[s+'false'],1)+' m');
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
for(const drive of ['awd','fwd','rwd'])for(const surface of ['tarmac','gravel','snow'])for(const vk of [40,70]){
  const scen=`catch ${drive} ${surface} ${vk}`;nTests++;const E=fresh({drive,surface});const S=E.S;setSpeed(E,vk/3.6);let t=0,bMax=0,recT=null;
  for(let i=0;i<Math.round(4/E.DT);i++){const b=Math.atan2(S.vy,Math.abs(S.vx));
    if(t<0.7){E.IN.steer=0.6;E.IN.thr=1;E.IN.hb=(drive==='fwd')?(t<0.4?1:0):0;}else{E.IN.hb=0;E.IN.steer=clamp(1.3*b/E.CAR.maxSteer,-1,1);E.IN.thr=0;}
    E.step(E.DT);t+=E.DT;bMax=Math.max(bMax,Math.abs(b)*DEG);if(t>0.7&&recT===null&&Math.abs(b)*DEG<5&&bMax>10)recT=t;}
  sane(E,scen);
  if(bMax>10&&recT===null)flag('WARN',scen,'does not recover (β max '+f(bMax,0)+'°)');
  if(bMax>85)flag('INFO',scen,'β max '+f(bMax,0)+'° despite counter-steer');
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
// ---------- 12) Profile: phantom lap, all drives ----------
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

// ---------- Output ----------
console.log('Consistency check:',nTests,'scenarios,',issues.length,'findings');
const order={ERROR:0,WARN:1,INFO:2};issues.sort((a,b)=>order[a.sev]-order[b.sev]);
for(const i of issues)console.log(' ',i.sev.padEnd(6),i.scen.padEnd(36),i.msg);
console.log('Notes:');for(const n of notes)console.log('  '+n);
