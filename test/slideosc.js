// Put the car directly into a big slide and watch for oscillations in yaw rate, rpm, wheel speed and forces.
const {loadEngine}=require('./load');const DEG=180/Math.PI;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=1)=>(+x).toFixed(d);
function run(lab,surface,beta0,vk,thr,steer,mode,kb){const E=loadEngine();Object.assign(E.cfg,{surface,drive:'rwd',auto:kb?true:false,steer:mode||'direct',uniform:true,profile:false,driven:false,mouse:false});E.VERT.amp=0;E.applyDriveSetup('rwd');E.selectTrack('rundkurs');const S=E.S,CAR=E.CAR;
  const v=vk/3.6,b=beta0/DEG;S.vx=v*Math.cos(b);S.vy=-v*Math.sin(b);S.r=0.9;S.psi=0;const w=S.vx/CAR.Rw;S.w=[w,w,w,w];S.gi=3;S.rpm=Math.max(CAR.idle,w*CAR.gears[3].r*CAR.final*9.55);S.we=S.rpm*0.10472;E.vertInit();
  let t=0;const rows=[];let prevR=S.r,flips=0,prevD=0;
  for(let i=0;i<Math.round(2.5/E.DT);i++){if(kb){E.keys['arrowup']=thr>0;E.keys['arrowright']=steer<0;E.keys['arrowleft']=steer>0;if(i%6===0)E.readInput(1/60);}else{E.IN.thr=thr;E.IN.steer=steer;}
    E.step(E.DT);t+=E.DT;const d=S.r-prevR;if(prevD*d<0&&Math.abs(d)>0.02)flips++;prevD=d;prevR=S.r;
    if(i%18===0)rows.push(f(t,2)+':r'+f(S.r*DEG,0)+'/b'+f(Math.atan2(S.vy,Math.abs(S.vx))*DEG,0)+'/rpm'+f(S.rpm,0)+'/wR'+f(S.w[3]*CAR.Rw*3.6,0)+'/thr'+f(E.IN.thr,1));}
  console.log(lab.padEnd(46),'yaw-rate reversals:',flips,'\n   '+rows.filter((x,i)=>i%2===0).join(' '));}
run('tarmac beta 50, 50 km/h, throttle 0, steer -0.6','tarmac',50,50,0,-0.6);
run('tarmac beta 50, 50 km/h, throttle 0.4','tarmac',50,50,0.4,-0.6);
run('tarmac beta 50, 50 km/h, throttle 1.0','tarmac',50,50,1.0,-0.6);
run('gravel beta 50, 50 km/h, throttle 0.4','gravel',50,50,0.4,-0.6);
run('tarmac beta 50, keyboard: up held, right held','tarmac',50,50,1,-1,'direct',true);
run('tarmac beta 50, keyboard, slip curve','tarmac',50,50,1,-1,'slip',true);
