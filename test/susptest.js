const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const clamp=(v,a,b)=>v<a?a:v>b?b:v;
const f=(x,d=2)=>(+x).toFixed(d);
function fresh(s,rough=0){const E=loadEngine();Object.assign(E.cfg,{surface:s,drive:'awd',abs:true,auto:true,steer:'direct',uniform:true});E.VERT.amp=rough;E.selectTrack('rundkurs');return E;}
function setSpeed(E,v){const S=E.S,CAR=E.CAR;S.vx=v;S.vy=0;S.r=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.Fxr=[0,0,0,0];S.Fyr=[0,0,0,0];S.gi=2;for(let g=6;g>=2;g--){const rpm=w*CAR.gears[g].r*CAR.final*60/(2*Math.PI);if(rpm>=2900){S.gi=g;break;}}S.rpm=w*CAR.gears[S.gi].r*CAR.final*60/(2*Math.PI);S.we=S.rpm*0.10472;S.shiftLock=0.4;E.vertInit();}
function hold(E,vT,c){const v=Math.hypot(E.S.vx,E.S.vy);const e=vT-v;c.I=clamp((c.I||0)+e*E.DT*0.6,-0.6,0.9);const u=0.35*e+c.I;E.IN.thr=clamp(u,0,1);E.IN.brk=clamp(-u*0.5,0,1);}
console.log('Engine:',process.env.ENGINE||'engine.js');
// 1) Landing: body with downward vertical velocity (jump from height h)
for(const h of [0.15,0.30,0.50]){const E=fresh('gravel');const V=E.VERT,S=E.S;setSpeed(E,20);const vz=-Math.sqrt(2*G*h);V.dz=vz;for(let i=0;i<4;i++)V.dzu[i]=vz;
  let dsMin=0,dsMax=0,FzMax=0,zMin=0,hits=0,tSettle=null;for(let i=0;i<720;i++){E.step(E.DT);dsMin=Math.min(dsMin,...V.d);dsMax=Math.max(dsMax,...V.d);FzMax=Math.max(FzMax,...S.Fz);zMin=Math.min(zMin,V.z);if(Math.min(...V.d)<-V.travelB)hits++;
    if(tSettle===null&&i>36&&Math.abs(V.dz)<0.05&&Math.abs(V.z)<0.01)tSettle=i*E.DT;}
  console.log('landing from',f(h,2),'m (vz',f(vz,1),'m/s): max compression',f(-dsMin*1000,0),'mm  max extension',f(dsMax*1000,0),'mm  in bump stop',f(hits*E.DT*1000,0),'ms  max wheel load',f(FzMax,0),'N  body min',f(zMin*1000,0),'mm  settled after',tSettle?f(tSettle,2)+' s':'–');}
// 2) Free heave decay: body pushed down 40 mm and released
{const E=fresh('tarmac');const V=E.VERT;setSpeed(E,20);V.z=-0.04;for(let i=0;i<4;i++)V.d[i]=V.z-V.zu[i];const pk=[];let prev=0,prevd=0;for(let i=0;i<720;i++){E.step(E.DT);const d=V.z-prev;if(prevd>0&&d<=0&&V.z>0.001)pk.push(V.z);if(prevd<0&&d>=0&&V.z<-0.001)pk.push(V.z);prev=V.z;prevd=d;}
  console.log('heave decay from -40 mm: extrema [mm]',pk.slice(0,5).map(x=>f(x*1000,1)).join(' → '),'  (ratio of successive half-waves ≈ '+(pk.length>1?f(Math.abs(pk[1]/pk[0]),2):'–')+')');}
// 3) Rough gravel 250 %, 110 km/h straight: travel usage, stops, wheel load
for(const [rough,vk] of [[1,110],[2.5,110],[2.5,140]]){const E=fresh('gravel',rough);const V=E.VERT,S=E.S;setSpeed(E,vk/3.6);const c={};let dsMin=0,dsMax=0,FzMin=1e9,FzMax=0,hitB=0,hitD=0,thMax=0,phMax=0,n=0,z0=0;
  for(let i=0;i<360*6;i++){hold(E,vk/3.6,c);E.step(E.DT);if(i<360)continue;n++;dsMin=Math.min(dsMin,...V.d);dsMax=Math.max(dsMax,...V.d);FzMin=Math.min(FzMin,...S.Fz);FzMax=Math.max(FzMax,...S.Fz);
    if(Math.min(...V.d)<-V.travelB)hitB++;if(Math.max(...V.d)>V.travelD)hitD++;thMax=Math.max(thMax,Math.abs(V.th));phMax=Math.max(phMax,Math.abs(V.ph));if(Math.min(...S.Fz)<50)z0++;}
  console.log('gravel rough',rough*100,'%,',vk,'km/h: travel',f(dsMin*1000,0),'…',f(dsMax*1000,0),'mm  bump stop',f(hitB/n*100,1),'%  droop stop',f(hitD/n*100,1),'%  wheel load',f(FzMin,0),'…',f(FzMax,0),'N  wheel airborne',f(z0/n*100,1),'%  pitch max',f(thMax*DEG,1),'°  roll max',f(phMax*DEG,1),'°');}
// 4) Brake dive: step to full brake, pitch-angle history (overshoot)
{const E=fresh('tarmac');const V=E.VERT,S=E.S;setSpeed(E,100/3.6);const c={};for(let i=0;i<360;i++){hold(E,100/3.6,c);E.step(E.DT);}E.IN.thr=0;E.IN.brk=1;
  let thMax=0,tMax=0,thSS=0;for(let i=0;i<540;i++){E.step(E.DT);if(Math.abs(V.th)>thMax){thMax=Math.abs(V.th);tMax=i*E.DT;}if(i>=400&&i<500)thSS+=Math.abs(V.th)/100;}
  console.log('brake dive: max',f(thMax*DEG,2),'° after',f(tMax,2),'s, steady ≈',f(thSS*DEG,2),'° → overshoot',f((thMax/thSS-1)*100,0),'%');}
