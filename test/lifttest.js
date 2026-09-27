const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=1)=>(+x).toFixed(d);
function setSpeed(E,v){const S=E.S,CAR=E.CAR;S.vx=v;S.vy=0;S.r=0;S.delta=0;S.dComp=0;S.psi=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.gi=2;for(let g=6;g>=2;g--){const rpm=w*CAR.gears[g].r*CAR.final*9.55;if(rpm>=2900){S.gi=g;break;}}S.rpm=w*CAR.gears[S.gi].r*CAR.final*9.55;S.we=S.rpm*0.10472;S.shiftLock=0.4;E.vertInit();}
function hold(E,vT,c){const e=vT-Math.hypot(E.S.vx,E.S.vy);c.I=clamp((c.I||0)+e*E.DT*0.6,-0.9,0.9);const u=0.35*e+c.I;E.IN.thr=clamp(u,0,1);E.IN.brk=clamp(-u*0.5,0,1);}
function go(drive,surface,ayT,action){const E=loadEngine();Object.assign(E.cfg,{drive,surface,auto:true,steer:'direct',uniform:true,profile:false,driven:false});E.VERT.amp=0;E.selectTrack('rundkurs');const S=E.S;const vT=70/3.6;setSpeed(E,vT);const c={};for(let i=0;i<720;i++){hold(E,vT,c);E.step(E.DT);}
  const d=((E.CAR.a+E.CAR.b)/(vT*vT)+(1.5/DEG)/G)*ayT*G;E.IN.steer=d/E.CAR.maxSteer;for(let i=0;i<1080;i++){hold(E,vT,c);E.step(E.DT);}
  const r0=S.r*DEG,ay0=S.ay/G;let bMax=0;const tr=[];for(let i=0;i<540;i++){E.IN.thr=action==='lift'?0:1;E.IN.brk=0;E.step(E.DT);bMax=Math.max(bMax,Math.abs(Math.atan2(S.vy,S.vx))*DEG);if(i%90===0)tr.push(f(i/360,2)+':r'+f(S.r*DEG,0)+'/β'+f(Math.atan2(S.vy,S.vx)*DEG,0)+'/κR'+f(S.kappa[2],2)+'/sR'+f(S.slip[2],1)+'/sF'+f(S.slip[0],1));}
  console.log((drive+' '+surface+' '+ayT+' g '+action).padEnd(30),'before r',f(r0,0),'ay',f(ay0),'  β max',f(bMax,0).padStart(3),'   ',tr.join(' '));}
console.log('Engine:',process.env.ENGINE||'engine.js');
go('rwd','tarmac',0.6,'lift');go('rwd','tarmac',0.6,'power');go('awd','tarmac',0.6,'lift');go('rwd','tarmac',0.4,'lift');
