const {loadEngine}=require('./load');const DEG=180/Math.PI;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=1)=>(+x).toFixed(d);
const E=loadEngine();Object.assign(E.cfg,{surface:'gravel',drive:'rwd',auto:true,steer:'direct',uniform:true,profile:false,driven:false});E.VERT.amp=0;E.applyDriveSetup('rwd');E.selectTrack('rundkurs');const S=E.S,CAR=E.CAR;
const v=40/3.6;S.vx=v;S.vy=0;S.r=0;S.psi=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.gi=3;S.rpm=w*CAR.gears[3].r*CAR.final*9.55;S.we=S.rpm*0.10472;E.vertInit();
let t=0;const o=[];for(let i=0;i<Math.round(3/E.DT);i++){const b=Math.atan2(S.vy,Math.abs(S.vx));if(t<0.7){E.IN.steer=0.6;E.IN.thr=1;}else{E.IN.steer=clamp(1.2*b/CAR.maxSteer,-1,1);E.IN.thr=0;}E.step(E.DT);t+=E.DT;
  if(i%54===0)o.push(f(t,2)+':β'+f(b*DEG,0)+'/r'+f(S.r*DEG,0)+'/κ'+f(S.kappa[2],2)+','+f(S.kappa[3],2)+'/Fy'+f(S.Fyr[2]+S.Fyr[3],0)+'/rpm'+f(S.rpm,0));}
console.log((process.env.ENGINE||'engine.js').padEnd(16),o.join(' '));
