// Road hazards: a puddle across the whole road and one on the right half only, driven through at several speeds;
// braking from 100 km/h over sand on tarmac; and how many patches each landscape and weather produces.
const {loadEngine}=require('./load');const DEG=180/Math.PI;const f=(x,d=1)=>(+x).toFixed(d);
function setup(surface,weather){const E=loadEngine();Object.assign(E.cfg,{surface,weather,abs:true,auto:true,uniform:true,profile:false,driven:false,hazards:'off'});E.VERT.amp=0;E.selectTrack('bremse');return E;}
function place(E,i0,len,n0,n1,type,depth){const T=E.TRACK;T.patches=[{i0,i1:i0+Math.round(len/2.5),n0,n1,type,depth,seed:1,s1:0,s2:0,s3:0}];T.patchAt=new Array(T.N);for(let i=i0;i<=i0+Math.round(len/2.5);i++)(T.patchAt[i%T.N]||(T.patchAt[i%T.N]=[])).push(T.patches[0]);}
function through(lab,vk,n0,n1,depth){const E=setup('tarmac','rain');const S=E.S,T=E.TRACK;place(E,120,14,n0,n1,'puddle',depth);
  let i=104;const p=T.pts[i];S.x=p.x;S.y=p.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=i;S.vx=vk/3.6;const w=S.vx/E.CAR.Rw;S.w=[w,w,w,w];S.gi=4;S.rpm=4000;S.we=419;E.vertInit();
  let t=0,minMu=1,dv=0,v0=null,yawMax=0,psi0=S.psi,hzSeen=false;
  for(let k=0;k<360*4;k++){E.IN.thr=0.35;E.IN.brk=0;E.IN.steer=0;const vb=Math.hypot(S.vx,S.vy);E.step(E.DT);t+=E.DT;
    if(S.wHz.some(h=>h==='puddle')){hzSeen=true;if(v0===null)v0=vb;const m=Math.min(...[0,1,2,3].map(i2=>E.hazardAt(S.wIdx[i2],S.wN[i2],vb).mu));minMu=Math.min(minMu,m);}
    yawMax=Math.max(yawMax,Math.abs(S.psi-psi0)*DEG);}
  console.log(lab.padEnd(36),'grip in the water',f(minMu*100,0)+' %','| heading change after',f(yawMax,1)+' deg',hzSeen?'':'(not reached)');}
console.log('Puddles (wet tarmac, straight, steering held straight, light throttle):');
for(const v of [50,80,100,120])through('full width, 10 mm, '+v+' km/h',v,-5,5,0.010);
for(const v of [80,110])through('right half only, 10 mm, '+v+' km/h',v,-5,-0.5,0.010);
through('full width, 3 mm, 110 km/h',110,-5,5,0.003);
console.log('Braking 100-0 km/h on tarmac (ABS):');
for(const [lab,type,depth] of [['clean','',0],['over 30 m of sand, 5 mm','sand',0.005],['over 30 m of sand, 20 mm','sand',0.02],['over 30 m of gravel spill','gravel',0],['over 30 m of snow','snow',0],['over 30 m of ice','ice',0]]){
  const E=setup('tarmac','clear');const S=E.S,T=E.TRACK;if(type)place(E,81,30,-6,6,type,depth);let i=80;const p=T.pts[i];S.x=p.x;S.y=p.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=i;S.vx=100/3.6;const w=S.vx/E.CAR.Rw;S.w=[w,w,w,w];S.gi=4;E.vertInit();
  const x0=S.x,y0=S.y;let t=0;while(Math.hypot(S.vx,S.vy)>0.3&&t<15){E.IN.thr=0;E.IN.brk=1;E.step(E.DT);t+=E.DT;}console.log('  '+lab.padEnd(30),f(Math.hypot(S.x-x0,S.y-y0),1),'m');}
console.log('Patches generated (light / heavy) on the stage:');
for(const [b,w,surf] of [['nordic','rain','tarmac'],['nordic','rain','gravel'],['desert','clear','tarmac'],['savanna','clear','gravel'],['alpine','clear','tarmac'],['winter','snowfall','snow'],['med','clear','tarmac']]){
  const r=[];for(const lvl of ['light','heavy']){const E=loadEngine();Object.assign(E.cfg,{biome:b,weather:w,surface:surf,hazards:lvl});E.selectTrack('etappe');const c={};for(const P of E.TRACK.patches)c[P.type]=(c[P.type]||0)+1;r.push(lvl+' '+JSON.stringify(c));}
  console.log('  '+(b+' '+w+' '+surf).padEnd(26),r.join(' | '));}
