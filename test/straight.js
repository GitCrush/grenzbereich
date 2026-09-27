const {loadEngine}=require('./load');const DEG=180/Math.PI;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=2)=>(+x).toFixed(d);
function setSpeed(E,v){const S=E.S,CAR=E.CAR;S.vx=v;S.vy=0;S.r=0;S.delta=0;S.dComp=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.Fxr=[0,0,0,0];S.Fyr=[0,0,0,0];S.gi=2;for(let g=6;g>=2;g--){const rpm=w*CAR.gears[g].r*CAR.final*60/(2*Math.PI);if(rpm>=2900){S.gi=g;break;}}S.rpm=w*CAR.gears[S.gi].r*CAR.final*60/(2*Math.PI);S.we=S.rpm*0.10472;S.shiftLock=0.4;E.vertInit();}
function hold(E,vT,c){const v=Math.hypot(E.S.vx,E.S.vy);const e=vT-v;c.I=clamp((c.I||0)+e*E.DT*0.6,-0.6,0.9);const u=0.35*e+c.I;E.IN.thr=clamp(u,0,1);E.IN.brk=clamp(-u*0.5,0,1);}
function run(lab,kin,amp,vk=100,surface='gravel'){const E=loadEngine();Object.assign(E.cfg,{surface,abs:true,auto:true,steer:'direct',uniform:true});if(kin)Object.assign(E.CAR,kin);E.VERT.amp=amp;E.selectTrack('rundkurs');setSpeed(E,vk/3.6);const c={};let psiMax=0,rMax=0;const tr=[];
 for(let i=0;i<3600;i++){E.IN.steer=0;hold(E,vk/3.6,c);E.step(E.DT);psiMax=Math.max(psiMax,Math.abs(E.S.psi));rMax=Math.max(rMax,Math.abs(E.S.r));if(i%360===0)tr.push(f(E.S.psi*DEG,1));}
 console.log(lab.padEnd(34),'ψ max',f(psiMax*DEG,2).padStart(6),'°  r max',f(rMax*DEG,1).padStart(5),'°/s  y',f(E.S.y,1).padStart(6),'m   ψ(t):',tr.join(' '));}
console.log('Engine:',process.env.ENGINE||'engine.js');
run('rough 100 %, default',null,1);
run('rough 100 %, kinematics off',{toeF:0,toeR:0,bsF:0,bsR:0},1);
run('rough 100 %, toe only',{bsF:0,bsR:0},1);
run('rough 100 %, rear bump steer only',{toeF:0,toeR:0,bsF:0},1);
run('rough 100 %, front bump steer only',{toeF:0,toeR:0,bsR:0},1);
run('smooth, default',null,0);
run('rough 100 %, default, 60 km/h',null,1,60);
run('rough 100 %, default, tarmac',null,1,100,'tarmac');
