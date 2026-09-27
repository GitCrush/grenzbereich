const {loadEngine}=require('./load');const DEG=180/Math.PI;const f=(x,d=1)=>(+x).toFixed(d);
function setSpeed(E,v){const S=E.S,CAR=E.CAR;S.vx=v;S.vy=0;S.r=0;S.delta=0;S.dComp=0;S.psi=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.gi=2;for(let g=6;g>=2;g--){const rpm=w*CAR.gears[g].r*CAR.final*9.55;if(rpm>=2900){S.gi=g;break;}}S.rpm=w*CAR.gears[S.gi].r*CAR.final*9.55;S.we=S.rpm*0.10472;S.shiftLock=0.4;E.vertInit();}
function run(lab,cfg,surface,vk,plan){const E=loadEngine();Object.assign(E.cfg,{surface,drive:'awd',auto:true,steer:'direct',uniform:true,profile:false,driven:false,abs:false},cfg);E.VERT.amp=0;E.selectTrack('rundkurs');const S=E.S;setSpeed(E,vk/3.6);
  let t=0,bMax=0;const tr=[];const brkRamp=(x,target)=>Math.min(target,target*x/0.8);
  for(const [dur,brk,steer,thr] of plan){const n=Math.round(dur/E.DT);let t0=t;for(let i=0;i<n;i++){E.IN.brk=brkRamp(t-t0,brk);if(brk===0)E.IN.brk=0;E.IN.steer=steer;E.IN.thr=thr||0;E.step(E.DT);t+=E.DT;const v=Math.hypot(S.vx,S.vy);if(v<2)break;const b=Math.atan2(S.vy,Math.abs(S.vx))*DEG;bMax=Math.max(bMax,Math.abs(b));
    if(Math.round(t/E.DT)%90===0)tr.push(f(t)+':v'+f(v*3.6,0)+'/β'+f(b,0)+'/r'+f(S.r*DEG,0)+'/κ'+f(S.kappa[0])+','+f(S.kappa[2]));}}
  console.log(lab.padEnd(50),'β max',f(bMax,0).padStart(3),'°  ',tr.join(' '));}
console.log('Engine:',process.env.ENGINE||'engine.js');
run('tarmac 100: full brake 1.2 s with steer 0.3, then release',{},'tarmac',100,[[1.2,1,0.3],[2.5,0,0.3]]);
run('tarmac 100: same, ABS',{abs:true},'tarmac',100,[[1.2,1,0.3],[2.5,0,0.3]]);
run('tarmac 100: same, centre 50 %',{clock:0.5},'tarmac',100,[[1.2,1,0.3],[2.5,0,0.3]]);
run('gravel 80: full brake 1.0 s with steer 0.3, then release',{},'gravel',80,[[1.0,1,0.3],[2.5,0,0.3]]);
run('gravel 80: same, centre 50 %',{clock:0.5},'gravel',80,[[1.0,1,0.3],[2.5,0,0.3]]);
run('gravel 80: full brake 1.0 s straight, then steer 0.3 + throttle',{},'gravel',80,[[1.0,1,0],[2.5,0,0.3,0.6]]);
run('tarmac 120: turn-in 0.2 first, then full brake',{},'tarmac',120,[[0.6,0,0.2],[2.5,1,0.2]]);
run('tarmac 120: same, centre 50 %',{clock:0.5},'tarmac',120,[[0.6,0,0.2],[2.5,1,0.2]]);
