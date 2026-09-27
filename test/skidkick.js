// Skidpad with a keyboard driver: throttle held flat, steering follows the circle; automatic upshift near the limit.
const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=1)=>(+x).toFixed(d);
function run(lab,drive,thrHold,mod){const E=loadEngine();Object.assign(E.cfg,{surface:'tarmac',drive,auto:true,steer:'direct',uniform:true,profile:false,driven:false});E.VERT.amp=0;E.applyDriveSetup(drive);E.selectTrack('kreis');mod&&mod(E);const S=E.S,T=E.TRACK,L=E.CAR.a+E.CAR.b;
  const p=T.pts[10];S.x=p.x;S.y=p.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=10;const v0=45/3.6;S.vx=v0;const w=v0/E.CAR.Rw;S.w=[w,w,w,w];S.gi=2;S.rpm=w*E.CAR.gears[2].r*E.CAR.final*9.55;S.we=S.rpm*0.10472;E.vertInit();
  let t=0,tShift=null,ayAt=0,bAfter=0,kAfter=0,spun=false;const o=[];
  for(let i=0;i<Math.round(10/E.DT);i++){let j=S.idx,acc=0;while(acc<10){acc+=T.pts[j].seg;j=(j+1)%T.N;}const q=T.pts[j];const al=Math.atan2(q.y-S.y,q.x-S.x)-S.psi;const a=Math.atan2(Math.sin(al),Math.cos(al));E.IN.steer=clamp(Math.atan(2*L*Math.sin(a)/10)/E.CAR.maxSteer,-1,1);
    E.IN.thr=thrHold;E.IN.brk=0;const g0=S.gi;E.step(E.DT);t+=E.DT;const b=Math.atan2(S.vy,Math.abs(S.vx))*DEG;
    if(S.gi!==g0&&tShift===null){tShift=t;ayAt=Math.abs(S.ay)/G;}
    if(tShift!==null&&t<tShift+1.5){bAfter=Math.max(bAfter,Math.abs(b));kAfter=Math.max(kAfter,S.kappa[2],S.kappa[3]);}
    if(tShift!==null&&Math.round((t-tShift)/E.DT)%36===0&&t-tShift<1.0)o.push(f(t-tShift,1)+':β'+f(b,0)+'/κ'+f(S.kappa[3],2)+'/thr'+f(E.IN.thr,1));
    if(Math.abs(b)>60){spun=true;break;}}
  console.log(lab.padEnd(44),tShift===null?'no shift':'shift at '+f(ayAt)+' g, v '+f(Math.hypot(S.vx,S.vy)*3.6,0)+' km/h',' β after',f(bAfter,0).padStart(3)+'°','κ rear max',f(kAfter,2),spun?' SPIN':'','   ',o.join(' '));}
run('RWD, keyboard flat (thr 1.0), ease 0.2/0.45 s','rwd',1.0);
run('RWD, thr 0.6','rwd',0.6);
run('RWD, thr 1.0, full lift 0.45 s','rwd',1.0,E=>{E.CAR.easeDeep=0;});
run('RWD, thr 1.0, lighter flywheel 0.15','rwd',1.0,E=>{E.CAR.Ieng=0.15;});
run('RWD, thr 1.0, flywheel 0.15 + full lift','rwd',1.0,E=>{E.CAR.Ieng=0.15;E.CAR.easeDeep=0;});
run('AWD, keyboard flat','awd',1.0);
run('AWD, thr 1.0, flywheel 0.15','awd',1.0,E=>{E.CAR.Ieng=0.15;});
