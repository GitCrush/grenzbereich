const {loadEngine}=require('./load');const DEG=180/Math.PI;const f=(x,d=1)=>(+x).toFixed(d);
function setSpeed(E,v){const S=E.S,CAR=E.CAR;S.vx=v;S.vy=0;S.r=0;S.delta=0;S.dComp=0;S.psi=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.gi=2;for(let g=6;g>=2;g--){const rpm=w*CAR.gears[g].r*CAR.final*9.55;if(rpm>=2900){S.gi=g;break;}}S.rpm=w*CAR.gears[S.gi].r*CAR.final*9.55;S.we=S.rpm*0.10472;S.shiftLock=0.4;E.vertInit();}
function run(lab,surface,vk,thr,steer,steerT){const E=loadEngine();Object.assign(E.cfg,{surface,auto:true,steer:'direct',uniform:true,profile:false,driven:false,abs:false});E.PED.threshold=thr;E.VERT.amp=0;E.selectTrack('rundkurs');const S=E.S;setSpeed(E,vk/3.6);
  let t=0,bMax=0,rMax=0,bEnd=0;for(let i=0;i<Math.round(4/E.DT);i++){E.keys['arrowdown']=true;if(i%6===0)E.readInput(1/60);E.IN.steer=t>steerT?steer:0;E.step(E.DT);t+=E.DT;const v=Math.hypot(S.vx,S.vy);if(v<3)break;bEnd=Math.atan2(S.vy,Math.abs(S.vx))*DEG;bMax=Math.max(bMax,Math.abs(bEnd));rMax=Math.max(rMax,Math.abs(S.r)*DEG);}
  console.log(lab.padEnd(46),'β max',f(bMax,0).padStart(3),'°  r max',f(rMax,0).padStart(3),'°/s');}
console.log('Keyboard full brake from 100 km/h (snow 80), steering from 0.8 s:');
for(const [surface,vk] of [['tarmac',100],['gravel',100],['snow',80]])for(const st of [0.15,0.3,1.0])for(const thr of [false,true])run(surface+' steer '+st+(thr?' threshold':' raw'),surface,vk,thr,st,0.8);
