// Review items M1, M4, M5, M6 (review of the vehicle model): the figures before and after a change.
// ENGINE=other-engine.js node review.js  runs the same probes on another extracted engine.
const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const f=(x,d=2)=>(+x).toFixed(d);
function fresh(cfg){const E=loadEngine();E.PED.threshold=false;Object.assign(E.cfg,{surface:'tarmac',drive:'awd',abs:false,tc:false,auto:true,steer:'direct',uniform:true,ret:false,profile:false,driven:false,hazards:'off',weather:'clear'},cfg||{});E.VERT.amp=0;E.applyDriveSetup(E.cfg.drive);E.selectTrack(cfg&&cfg.track||'bremse');return E;}
function at(E,i,v,gi){const S=E.S,T=E.TRACK,p=T.pts[i];S.x=p.x;S.y=p.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=i;S.vx=v;S.vy=0;S.r=0;const w=v/E.CAR.Rw;S.w=[w,w,w,w];S.gi=gi||4;S.rpm=4000;S.we=419;E.vertInit();}

// M1: full braking with ABS on a 15 % grade (a Gaussian hill on a long straight, measured at its inflection, where the
// grade is steepest and the vertical curvature zero) against the same on the flat. The tyre deceleration should
// follow the wheel load, m·g·cos θ: ratio ≈ 0.99 up and down.
const RAMP={name:'Ramp',type:'seg',segs:[{s:900},{R:60,a:180},{s:900},{R:60,a:180}],w:7,prof:{z:[[450,43.7,250]],bank:[]}};
function brakeOnGrade(prof,sStart){const E=fresh({abs:true,profile:prof});E.TRACKS.ramp=RAMP;E.selectTrack('ramp');const S=E.S,T=E.TRACK;
  let i=0;while(T.pts[i].s<sStart)i++;at(E,i,100/3.6);let t=0,sum=0,n=0,gr=0;
  while(t<1.2){E.IN.thr=0;E.IN.brk=1;E.IN.steer=0;E.step(E.DT);t+=E.DT;if(t>0.4){sum+=-S.ax;gr+=S.slopeX;n++;}}
  return {a:sum/n,grade:gr/n};}
const flat=brakeOnGrade(false,600),down=brakeOnGrade(true,608),up=brakeOnGrade(true,250);
console.log('M1  braking with ABS, tyre deceleration: flat',f(flat.a),'m/s² | downhill',f(down.grade*100,1)+' %',f(down.a),'(×'+f(down.a/flat.a,3)+')',
  '| uphill',f(up.grade*100,1)+' %',f(up.a),'(×'+f(up.a/flat.a,3)+')  expected ×'+f(Math.cos(Math.atan(0.15)),3));

// M4: steering ramp at 60 km/h on the R 40 m skidpad track (as the matrix does): lateral limit on sand against gravel.
function ramp(cfg,vk){const E=fresh(Object.assign({track:'kreis'},cfg));const S=E.S;const vT=vk/3.6;at(E,0,vT,3);
  const c={};const hold=()=>{const e=vT-Math.hypot(S.vx,S.vy);c.I=Math.max(-0.9,Math.min(0.9,(c.I||0)+e*E.DT*0.6));const u=0.35*e+c.I;E.IN.thr=Math.max(0,Math.min(1,u));E.IN.brk=Math.max(0,Math.min(1,-u*0.5));};
  for(let k=0;k<720;k++){E.IN.steer=0;hold();E.step(E.DT);}
  let t=0,ayMax=0,dAt=0;while(t<50){const d=0.5/DEG*t;E.IN.steer=d/E.CAR.maxSteer;hold();E.step(E.DT);t+=E.DT;const ay=Math.abs(S.ay);if(ay>ayMax){ayMax=ay;dAt=d*DEG;}if(ay<0.9*ayMax&&d*DEG>dAt+2)break;}
  return ayMax/G;}
const gr=ramp({surface:'gravel'},60),sa=ramp({surface:'sand'},60);
console.log('M4  steering ramp 60 km/h, ay,max: gravel',f(gr),'g, sand',f(sa),'g, sand/gravel',f(sa/gr),'(µ ratio '+f(0.66/0.88)+')');

// M5: wet against dry tarmac, same ramp at 60 and at 110 km/h – the wet grip should fall with speed.
const d60=ramp({surface:'tarmac'},60),w60=ramp({surface:'tarmac',weather:'rain'},60),d110=ramp({surface:'tarmac'},110),w110=ramp({surface:'tarmac',weather:'rain'},110);
console.log('M5  wet/dry ay,max: 60 km/h',f(w60)+'/'+f(d60),'= ×'+f(w60/d60),'| 110 km/h',f(w110)+'/'+f(d110),'= ×'+f(w110/d110));

// M6: a 10 mm puddle across the whole road on gravel and on tarmac at 100 km/h, light throttle, steering straight:
// least grip factor in the water and the speed lost, rolling freely (in neutral would be cleaner; off the throttle is close). On loose ground: mud, no dynamic hydroplaning.
function puddle(surface,vk,depth){const E=fresh({surface,weather:'rain',abs:true});const S=E.S,T=E.TRACK;
  T.patches=[{i0:120,i1:126,n0:-5,n1:5,type:'puddle',depth,seed:1,s1:0,s2:0,s3:0}];T.patchAt=new Array(T.N);for(let i=120;i<=126;i++)(T.patchAt[i]||(T.patchAt[i]=[])).push(T.patches[0]);
  at(E,104,vk/3.6);let minMu=1,vIn=null,vOut=null,wMax=0;
  for(let k=0;k<360*3;k++){E.IN.thr=0;E.IN.brk=0;E.IN.steer=0;const vb=Math.hypot(S.vx,S.vy);E.step(E.DT);
    const inW=S.wHz.some(h=>h==='puddle');if(inW){if(vIn===null)vIn=vb;vOut=Math.hypot(S.vx,S.vy);minMu=Math.min(minMu,...[0,1,2,3].map(i=>E.hazardAt(S.wIdx[i],S.wN[i],vb).mu));wMax=Math.max(wMax,S.w[0]*E.CAR.Rw/vb);}}
  return {minMu,dv:(vIn-vOut)*3.6,wMax};}
for(const s of ['tarmac','gravel','sand']){const p=puddle(s,100,0.010),q=puddle(s,60,0.010);
  console.log('M6  10 mm puddle on '+s.padEnd(6)+': 100 km/h grip ×'+f(p.minMu)+', speed lost '+f(p.dv,1)+' km/h | 60 km/h grip ×'+f(q.minMu)+', lost '+f(q.dv,1)+' km/h');}
