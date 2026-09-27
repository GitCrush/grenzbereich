const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=2)=>(+x).toFixed(d);
function fresh(s,c){const E=loadEngine();Object.assign(E.cfg,{surface:s,drive:'awd',abs:false,auto:true,steer:'direct',uniform:true},c||{});E.VERT.amp=0;E.selectTrack('rundkurs');return E;}
function setSpeed(E,v){const S=E.S,CAR=E.CAR;S.vx=v;S.vy=0;S.r=0;S.delta=0;S.dComp=0;S.psi=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.Fxr=[0,0,0,0];S.Fyr=[0,0,0,0];S.gi=2;for(let g=6;g>=2;g--){const rpm=w*CAR.gears[g].r*CAR.final*60/(2*Math.PI);if(rpm>=2900){S.gi=g;break;}}S.rpm=w*CAR.gears[S.gi].r*CAR.final*60/(2*Math.PI);S.we=S.rpm*0.10472;S.shiftLock=0.4;E.vertInit();}
function hold(E,vT,c){const v=Math.hypot(E.S.vx,E.S.vy);const e=vT-v;c.I=clamp((c.I||0)+e*E.DT*0.6,-0.6,0.9);const u=0.35*e+c.I;E.IN.thr=clamp(u,0,1);E.IN.brk=clamp(-u*0.5,0,1);}
console.log('Engine:',process.env.ENGINE||'engine.js');
// 1) Tight corner, wind-up: R ≈ 12 m at 20 km/h, tarmac
console.log('--- Wind-up in a tight corner (20 km/h, δ = 12°, tarmac): κ front/rear, centre torque, throttle to hold ---');
for(const clock of [1,0.5,0]){const E=fresh('tarmac',{clock});const S=E.S;setSpeed(E,20/3.6);const c={};E.IN.steer=(12/DEG)/E.CAR.maxSteer;for(let i=0;i<1800;i++){hold(E,20/3.6,c);E.step(E.DT);}
  console.log('  lock',(clock*100).toFixed(0).padStart(3),'%: κ front',f((S.kappa[0]+S.kappa[1])/2,3),' κ rear',f((S.kappa[2]+S.kappa[3])/2,3),'  Tcenter',f(S.Tcenter||0,0),'Nm  throttle',f(E.IN.thr,2),'  yaw rate',f(S.r*DEG,1),'°/s  R =',f(Math.hypot(S.vx,S.vy)/S.r,1),'m');}
// 2) Throttle stab in a steady corner (gravel, 60 km/h, 0.5 g): yaw-rate change after 1 s, inside-rear slip
console.log('--- Throttle stab to 100 % in a steady corner, gravel 60 km/h ≈ 0.5 g ---');
for(const [lab,c] of [['axle lock 0 %',{lsd:0}],['axle lock 55 %',{lsd:0.55}],['axle lock 100 %',{lsd:1}],['centre open 50/50, axle 55 %',{clock:0,split:0.5}],['centre open 40/60, axle 55 %',{clock:0,split:0.4}]]){
  const E=fresh('gravel',c);const S=E.S;setSpeed(E,60/3.6);const hc={};const L=E.CAR.a+E.CAR.b;const d=(L/(16.67*16.67)+(3.3/DEG)/G)*0.5*G;E.IN.steer=d/E.CAR.maxSteer;for(let i=0;i<1440;i++){hold(E,60/3.6,hc);E.step(E.DT);}
  const r0=S.r,ay0=S.ay;E.IN.thr=1;E.IN.brk=0;let kIn=0,kOut=0,rMax=r0,rMin=r0;for(let i=0;i<360;i++){E.step(E.DT);kIn=Math.max(kIn,S.kappa[2]);kOut=Math.max(kOut,S.kappa[3]);rMax=Math.max(rMax,S.r);rMin=Math.min(rMin,S.r);}
  console.log('  '+lab.padEnd(30),'r before',f(r0*DEG,1),'°/s → after 1 s',f(S.r*DEG,1),'(min',f(rMin*DEG,1),'max',f(rMax*DEG,1),')  κ rear inside/outside max',f(kIn,2),'/',f(kOut,2),'  β',f(Math.atan2(S.vy,S.vx)*DEG,1),'°');}
// 3) Lift-off in a steady corner (gravel 60 km/h, 0.5 g), coast side of the diff
console.log('--- Lift-off in a steady corner, gravel 60 km/h ≈ 0.5 g ---');
for(const [lab,c] of [['coast lock 0 %',{lsdCoast:0}],['coast lock 30 %',{lsdCoast:0.3}],['coast lock 80 %',{lsdCoast:0.8}]]){
  const E=fresh('gravel',c);const S=E.S;setSpeed(E,60/3.6);const hc={};const L=E.CAR.a+E.CAR.b;const d=(L/(16.67*16.67)+(3.3/DEG)/G)*0.5*G;E.IN.steer=d/E.CAR.maxSteer;for(let i=0;i<1440;i++){hold(E,60/3.6,hc);E.step(E.DT);}
  const r0=S.r;E.IN.thr=0;E.IN.brk=0;let rMax=r0;for(let i=0;i<360;i++){E.step(E.DT);rMax=Math.max(rMax,S.r);}
  console.log('  '+lab.padEnd(30),'r before',f(r0*DEG,1),'°/s → max within 1 s',f(rMax*DEG,1),'  after 1 s',f(S.r*DEG,1),'  β',f(Math.atan2(S.vy,S.vx)*DEG,1),'°');}
// 4) Upshift 1→2 at full throttle, tarmac: slip peak, rpm history
{const E=fresh('tarmac');const S=E.S;E.IN.thr=1;let t=0,tr=[],kPk=0,tS=null,vAt=0;while(t<4){E.step(E.DT);t+=E.DT;if(S.gi===3&&tS===null){tS=t;vAt=Math.hypot(S.vx,S.vy)*3.6;}if(tS!==null&&t<tS+0.4){kPk=Math.max(kPk,S.kappa[2]);if(Math.round((t-tS)/E.DT)%9===0)tr.push(S.rpm.toFixed(0));}}
  console.log('--- Upshift 1→2 full throttle tarmac at',f(vAt,0),'km/h: κ rear max within 0.4 s',f(kPk,2),'  rpm every 25 ms:',tr.join(' '));}
