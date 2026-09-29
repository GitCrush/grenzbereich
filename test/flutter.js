// Frame-to-frame jitter of the HUD channels at steady speed: current engine vs the pre-constraint engine.
const {loadEngine}=require('./load');const f=(x,d=3)=>(+x).toFixed(d);
function run(lab,surface,vk,rough,steer,keyboard){const E=loadEngine();Object.assign(E.cfg,{surface,drive:'awd',auto:true,steer:'direct',uniform:true,profile:false,driven:false,mouse:false});E.VERT.amp=rough;E.selectTrack('bremse');const S=E.S,CAR=E.CAR;
  const v=vk/3.6;S.vx=v;const w=v/CAR.Rw;S.w=[w,w,w,w];S.gi=4;S.rpm=w*CAR.gears[4].r*CAR.final*9.55;S.we=S.rpm*0.10472;E.vertInit();
  const ch={kFL:[],kRL:[],FzFL:[],useF:[],useR:[],thr:[],ay:[],rpm:[]};let c={I:0};
  for(let i=0;i<360*6;i++){if(keyboard){E.keys['arrowup']=true;if(i%6===0)E.readInput(1/60);}else{const e=v-Math.hypot(S.vx,S.vy);c.I=Math.max(-0.5,Math.min(0.9,c.I+e*E.DT*0.6));E.IN.thr=Math.max(0,Math.min(1,0.35*e+c.I));}
    E.IN.steer=steer;E.step(E.DT);
    if(i>360*2&&i%6===0){ch.kFL.push(S.kappa[0]);ch.kRL.push(S.kappa[2]);ch.FzFL.push(S.Fz[0]);ch.useF.push(Math.max(S.slip[0],S.slip[1]));ch.useR.push(Math.max(S.slip[2],S.slip[3]));ch.thr.push(E.IN.thr);ch.ay.push(S.ay);ch.rpm.push(S.rpm);}}
  const jit=a=>{let s=0;for(let i=1;i<a.length;i++)s+=Math.abs(a[i]-a[i-1]);return s/(a.length-1);};
  console.log(lab.padEnd(44),Object.entries(ch).map(([k,a])=>k+' '+f(jit(a),k==='FzFL'||k==='rpm'?0:3)).join('  '));}
console.log((process.env.ENGINE||'engine.js')+': mean |frame-to-frame change| at 60 Hz');
run('tarmac 80 km/h smooth, PI throttle','tarmac',80,0,0,false);
run('tarmac 80 km/h rough, PI throttle','tarmac',80,1,0,false);
run('gravel 80 km/h rough, PI throttle','gravel',80,1,0,false);
run('gravel 80 km/h rough, keyboard throttle','gravel',80,1,0,true);
run('gravel 60 km/h rough, steer 0.15, keyboard','gravel',60,1,0.15,true);
