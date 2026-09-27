const {loadEngine}=require('./load');const DEG=180/Math.PI;const f=(x,d=1)=>(+x).toFixed(d);
function setSpeed(E,v){const S=E.S,CAR=E.CAR;S.vx=v;S.vy=0;S.r=0;S.delta=0;S.dComp=0;S.psi=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.gi=2;for(let g=6;g>=2;g--){const rpm=w*CAR.gears[g].r*CAR.final*9.55;if(rpm>=2900){S.gi=g;break;}}S.rpm=w*CAR.gears[S.gi].r*CAR.final*9.55;S.we=S.rpm*0.10472;S.shiftLock=0.4;E.vertInit();}
function run(lab,cfg,brk,steerT,steer,vk=120,kb=true){const E=loadEngine();Object.assign(E.cfg,{surface:'tarmac',drive:'awd',auto:true,steer:'direct',uniform:true,profile:false,driven:false,abs:false},cfg);E.VERT.amp=0;E.selectTrack('rundkurs');const S=E.S;setSpeed(E,vk/3.6);
  let t=0,bMax=0,rMax=0;const tr=[];const brkRamp=(t)=>Math.min(brk,brk*t/0.8);   // keyboard ramp
  for(let i=0;i<Math.round(3.5/E.DT);i++){E.IN.thr=0;E.IN.brk=kb?brkRamp(t):brk;E.IN.steer=t>steerT?steer:0;E.step(E.DT);t+=E.DT;const b=Math.atan2(S.vy,Math.abs(S.vx))*DEG;bMax=Math.max(bMax,Math.abs(b));rMax=Math.max(rMax,Math.abs(S.r)*DEG);
    if(Math.round(t/E.DT)%90===0)tr.push(f(t)+':v'+f(Math.hypot(S.vx,S.vy)*3.6,0)+'/β'+f(b,0)+'/κF'+f((S.kappa[0]+S.kappa[1])/2)+'/κR'+f((S.kappa[2]+S.kappa[3])/2)+'/FzR'+f(S.Fz[2]+S.Fz[3],0));}
  console.log(lab.padEnd(44),'β max',f(bMax,0).padStart(3),'°  r max',f(rMax,0).padStart(3),'°/s   ',tr.join(' '));}
console.log('Engine:',process.env.ENGINE||'engine.js','— 120 km/h tarmac, brake (keyboard ramp 0.8 s), steer 0.15 from 0.5 s');
run('full brake, centre rigid',{},1,0.5,0.15);
run('full brake, centre 50 % plated',{clock:0.5},1,0.5,0.15);
run('full brake, centre open',{clock:0},1,0.5,0.15);
run('full brake, ABS',{abs:true},1,0.5,0.15);
run('brake 60 %, centre rigid',{},0.6,0.5,0.15);
run('brake 60 %, centre 50 %',{clock:0.5},0.6,0.5,0.15);
run('full brake straight, centre rigid',{},1,9,0);
run('full brake, bias 60 % front, rigid',{bias:0.60},1,0.5,0.15);
run('full brake, bias 85 % front, rigid',{bias:0.85},1,0.5,0.15);
