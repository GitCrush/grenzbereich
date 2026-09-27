const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=2)=>(+x).toFixed(d);
function fresh(s,kin){const E=loadEngine();Object.assign(E.cfg,{surface:s,drive:'awd',abs:true,auto:true,steer:'direct',uniform:true});Object.assign(E.CAR,kin||{});E.VERT.amp=0;E.selectTrack('rundkurs');return E;}
function setSpeed(E,v){const S=E.S,CAR=E.CAR;S.vx=v;S.vy=0;S.r=0;S.delta=0;S.dComp=0;const w=v/CAR.Rw;S.w=[w,w,w,w];S.Fxr=[0,0,0,0];S.Fyr=[0,0,0,0];S.gi=2;for(let g=6;g>=2;g--){const rpm=w*CAR.gears[g].r*CAR.final*60/(2*Math.PI);if(rpm>=2900){S.gi=g;break;}}S.rpm=w*CAR.gears[S.gi].r*CAR.final*60/(2*Math.PI);S.we=S.rpm*0.10472;S.shiftLock=0.4;E.vertInit();}
function hold(E,vT,c){const v=Math.hypot(E.S.vx,E.S.vy);const e=vT-v;c.I=clamp((c.I||0)+e*E.DT*0.6,-0.6,0.9);const u=0.35*e+c.I;E.IN.thr=clamp(u,0,1);E.IN.brk=clamp(-u*0.5,0,1);}
function ramp(E,vT){const S=E.S,L=E.CAR.a+E.CAR.b;setSpeed(E,vT);const c={};for(let i=0;i<720;i++){hold(E,vT,c);E.step(E.DT);}let t=0,ayMax=0,best=null;const lin=[];
  while(t<50){const d=0.5/DEG*t;E.IN.steer=d/E.CAR.maxSteer;hold(E,vT,c);E.step(E.DT);t+=E.DT;if(S.ay>0.1*G&&S.ay<0.3*G)lin.push([S.ay,d]);
   if(S.ay>ayMax){ayMax=S.ay;best={d:d*DEG,ay:S.ay/G,aF:(S.alpha[0]+S.alpha[1])/2*DEG,aR:(S.alpha[2]+S.alpha[3])/2*DEG,toeR:(S.toe[2]-S.toe[3])/2*DEG};}
   if(best&&S.ay<0.9*ayMax&&d*DEG>best.d+2)break;}
  const n=lin.length;let sx=0,sy=0,sxx=0,sxy=0;for(const [a,d] of lin){sx+=a;sy+=d;sxx+=a*a;sxy+=a*d;}
  return {...best,K:((n*sxy-sx*sy)/(n*sxx-sx*sx)-L/(vT*vT))*G*DEG};}
function stepS(E,vT,ayT,K){const S=E.S,L=E.CAR.a+E.CAR.b;setSpeed(E,vT);const c={};for(let i=0;i<720;i++){hold(E,vT,c);E.step(E.DT);}const d=(L/(vT*vT)+(K/DEG)/G)*ayT*G;E.IN.steer=d/E.CAR.maxSteer;
  const rs=[];for(let i=0;i<1080;i++){hold(E,vT,c);E.step(E.DT);rs.push(S.r);}const ss=rs.slice(900).reduce((a,b)=>a+b)/180;let t90=null,rMax=-1;for(let i=0;i<rs.length;i++){if(t90===null&&rs[i]>=0.9*ss)t90=i*E.DT;rMax=Math.max(rMax,rs[i]);}
  return {t90,over:(rMax/ss-1)*100};}
console.log('rear roll steer [°/°]  toe f/r [°] | K [°/g]  ay,max  αF   αR   δ@max  | step steer 80 km/h: t90   overshoot');
for(const [lab,kin] of [['no kinematics',{toeF:0,toeR:0,bsF:0,bsR:0}],['default',{}],['roll steer 0',{bsR:0}],['roll steer 0.20',{bsR:0.2*2/1.58}],['roll steer −0.10 (roll oversteer)',{bsR:-0.1*2/1.58}],['rear toe +0.5°',{toeR:0.5/DEG}],['front toe-out −0.5°',{toeF:-0.5/DEG}]]){
  const r=ramp(fresh('tarmac',kin),60/3.6);const st=stepS(fresh('tarmac',kin),80/3.6,0.4,r.K);const rg=ramp(fresh('gravel',kin),60/3.6);
  console.log(lab.padEnd(36),'| tarmac',f(r.K).padStart(5),' ',f(r.ay),' ',f(r.aF,1),f(r.aR,1),f(r.d,1),' |',f(st.t90,3),' ',f(st.over,1).padStart(5),'%  | gravel K',f(rg.K),' ay',f(rg.ay));}
// straight-line: 10 s on rough gravel 100 %, zero steering
{const E=loadEngine();Object.assign(E.cfg,{surface:'gravel',abs:true,auto:true,steer:'direct',uniform:true});E.VERT.amp=1;E.selectTrack('rundkurs');setSpeed(E,100/3.6);const c={};let psiMax=0;for(let i=0;i<3600;i++){E.IN.steer=0;hold(E,100/3.6,c);E.step(E.DT);psiMax=Math.max(psiMax,Math.abs(E.S.psi));}
 console.log('straight 10 s rough gravel: max yaw angle',f(psiMax*DEG,2),'°  lateral offset',f(E.S.y,2),'m');}
