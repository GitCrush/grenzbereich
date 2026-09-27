// Skidpad with the real keyboard path (arrow up held): metered vs raw throttle, RWD and AWD.
const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=1)=>(+x).toFixed(d);
function run(lab,drive,meter){const E=loadEngine();Object.assign(E.cfg,{surface:'tarmac',drive,auto:true,steer:'direct',uniform:true,profile:false,driven:false,mouse:false});E.VERT.amp=0;E.PED.meter=meter;E.applyDriveSetup(drive);E.selectTrack('kreis');const S=E.S,T=E.TRACK,L=E.CAR.a+E.CAR.b;
  const p=T.pts[10];S.x=p.x;S.y=p.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=10;const v0=45/3.6;S.vx=v0;const w=v0/E.CAR.Rw;S.w=[w,w,w,w];S.gi=2;S.rpm=w*E.CAR.gears[2].r*E.CAR.final*9.55;S.we=S.rpm*0.10472;E.vertInit();
  let t=0,tShift=null,ayAt=0,bMax=0,spun=false,vMax=0;const o=[];
  for(let i=0;i<Math.round(14/E.DT);i++){let j=S.idx,acc=0;while(acc<10){acc+=T.pts[j].seg;j=(j+1)%T.N;}const q=T.pts[j];const al=Math.atan2(q.y-S.y,q.x-S.x)-S.psi;const a=Math.atan2(Math.sin(al),Math.cos(al));
    E.keys['arrowup']=true;if(i%6===0)E.readInput(1/60);E.IN.steer=clamp(Math.atan(2*L*Math.sin(a)/10)/E.CAR.maxSteer,-1,1);
    const g0=S.gi;E.step(E.DT);t+=E.DT;const b=Math.atan2(S.vy,Math.abs(S.vx))*DEG;vMax=Math.max(vMax,Math.hypot(S.vx,S.vy)*3.6);
    if(S.gi!==g0&&tShift===null){tShift=t;ayAt=Math.abs(S.ay)/G;}if(tShift!==null&&t<tShift+1.5)bMax=Math.max(bMax,Math.abs(b));
    if(i%360===0&&t>1)o.push(f(t,0)+'s:v'+f(Math.hypot(S.vx,S.vy)*3.6,0)+'/ay'+f(Math.abs(S.ay)/G)+'/g'+E.CAR.gears[S.gi].n+'/thr'+f(E.IN.thr,2)+'/b'+f(b,0));
    if(Math.abs(b)>60){spun=true;break;}}
  console.log(lab.padEnd(26),tShift===null?'no shift':'shift at '+f(ayAt)+' g',' beta max after',f(bMax,0).padStart(3),'deg',spun?'SPIN at '+f(t,1)+' s':'no spin','  v max',f(vMax,0),'  ',o.join(' '));}
run('RWD keyboard metered','rwd',true);run('RWD keyboard raw','rwd',false);run('AWD keyboard metered','awd',true);run('AWD keyboard raw','awd',false);
