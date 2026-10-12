// Part 3 of the hybrid's go/no-go test (decision 11 Oct): the engine itself as the judge. In the measurement's world
// (ring of radius R, smooth road, LINE = null – as the identification), per held table point:
//  (1) a clean hold – the table's regulator, no excitation, 5 s; the means over the last 2 s are the engine's steady
//      slide (the table's operating point was measured under ±3° / ±0.08 excitation);
//  (2) steps from that hold – the state saved, then 0.5 s open loop: the regulator's last command ± 1° (the hands'
//      target, through the driver's chain), or its throttle ± 0.05; the response is (y+ − y−)/2 (even-order terms and
//      the slide's free drift cancel). Recorded at 60 Hz: β, r, v, δ, p, throttle.
// Written with the physical core's steady slide and linearisation at the hold's own radius and slip, for hybrid/eng.py.
// node hybrid/engstep.js OUT.json TAB.json [TAB.json …]   env: DRIVE, ONLY="R/b,R/b"
const {loadEngine}=require('../load');const {makeModel}=require('./phys');const fs=require('fs');
const E=loadEngine();const drive=process.env.DRIVE||'awd';E.cfg.drive=drive;E.applyDriveSetup(drive);E.selectTrack('rundkurs');
if(process.env.NORUT)E.ROAD[E.cfg.surface].rut=0;   // NORUT=1: the ruts off (the core leaves them out)
const P=makeModel(E,{ruts:!process.env.NORUT}),CAR=E.CAR,DT=E.DT,clamp=(v,a,b)=>v<a?a:v>b?b:v;
function tables(file){const d=JSON.parse(fs.readFileSync(file,'utf8'));if(d.tab)return [d.tab];
  return Object.entries(d).filter(([k])=>k.startsWith('gb.dc.')&&k.includes('|'+drive+'|')).map(([,v])=>JSON.parse(v));}
const tabs=process.argv.slice(3).flatMap(tables),gearOf=v=>Math.max(2,Math.min(CAR.gears.length-1,2+Math.floor(v/9)));
const only=process.env.ONLY?process.env.ONLY.split(','):null;
const base={S:E.cloneState(E.S),VERT:E.cloneState(E.VERT)},out=[];
for(const [ti,T] of tabs.entries())for(const M of T.pts){if(!M.ok||(only&&!only.includes(M.R+'/'+Math.round(M.b*57.3))))continue;
  if(M.Kdu)M.K.du=true;const ch=E.dcChain(),R=M.R,TR=E.buildTrack({name:'ring',type:'seg',segs:[{R,a:360}],w:30,ex:null});TR.patches=[];TR.patchAt=new Array(TR.N);
  const job={S:E.cloneState(base.S),IN:{steer:0,thr:0,brk:0,hb:0},VERT:E.cloneState(base.VERT),TRACK:TR};E.mjSwap(job,true);let rec=null;
  try{const S=E.S;let cx=0,cy=0;for(const p of TR.pts){cx+=p.x;cy+=p.y;}cx/=TR.N;cy/=TR.N;const ks=TR.pts[10].ks||1,d=-ks;
    S.noLead=true;E.VERT.amp=0;
    {const p0=TR.pts[10],v0=M.x0[2],bs=-d*M.x0[0];S.x=p0.x;S.y=p0.y;S.psi=Math.atan2(p0.ty,p0.tx)-bs;S.idx=10;S.vx=v0*Math.cos(bs);S.vy=v0*Math.sin(bs);S.r=-d*v0/R;
     S.delta=-d*M.x0[5];S.dDelta=0;S.hand=S.delta*CAR.ratio;S.dq=null;const w=S.vx/CAR.Rw;S.w=[w,w,w,w];
     S.gi=gearOf(v0);S.rpm=Math.max(1200,w*CAR.gears[S.gi].r*CAR.final*9.55);S.we=S.rpm*0.10472;E.vertInit();}
    const obs=()=>{const S=E.S,v=Math.hypot(S.vx,S.vy),be=Math.atan2(S.vy,Math.abs(S.vx)),e=Math.hypot(S.x-cx,S.y-cy)-R,chi=S.psi+Math.atan2(S.vy,S.vx),th=Math.atan2(S.y-cy,S.x-cx)+ks*Math.PI/2;
      let h=chi-th;h=Math.atan2(Math.sin(h),Math.cos(h));return [-d*be,-d*S.r,v,e,-d*h,-d*S.delta,-d*(S.dLP===undefined?S.delta:S.dLP)];};
    const C1=E.dcCtl(M,ch,M.K);let t=0,k=0,c1=E.S.delta,t1=M.u0[1],lost=false;const H=[];
    while(t<5){const x=obs();if(k%6===0){const u=C1.step(x.slice(0,6),x[6],{b:M.x0[0],e:0},null,null);c1=-d*u[0];t1=u[1];if(t>3)H.push([...x,t1,E.S.gi]);}
      E.IN.steer=clamp(c1/CAR.maxSteer,-1,1);E.IN.thr=t1;E.IN.brk=0;E.IN.hb=0;E.step(DT);t+=DT;k++;if(Math.abs(x[0])>1.2||Math.abs(x[3])>28){lost=true;break;}}
    if(lost){console.log(R,Math.round(M.b*57.3),'t'+ti,'lost in the hold');continue;}
    const mean=H[0].map((_,j)=>H.reduce((a,r)=>a+r[j],0)/H.length),sdv=H[0].map((_,j)=>Math.sqrt(H.reduce((a,r)=>a+(r[j]-mean[j])**2,0)/H.length));
    const snap={S:E.cloneState(E.S),VERT:E.cloneState(E.VERT)},gi=E.S.gi;
    const run=(dc,dthr)=>{E.S=E.cloneState(snap.S);Object.assign(E.VERT,E.cloneState(snap.VERT));E.IN={steer:0,thr:0,brk:0,hb:0,clutch:false};const Y=[obs()];
      for(let f=0;f<30;f++){for(let j=0;j<6;j++){E.IN.steer=clamp((c1-d*dc)/CAR.maxSteer,-1,1);E.IN.thr=clamp(t1+dthr,0,1);E.step(DT);}Y.push(obs());}return Y;};
    const half=(a,b)=>a.map((r,k)=>r.map((v,j)=>(v-b[k][j])/2));
    const dS=half(run(1/57.3,0),run(-1/57.3,0)),dT=half(run(0,0.05),run(0,-0.05));
    rec={ti,R,b:Math.round(M.b*57.3),gi,mean,sd:sdv,c1:-d*c1,t1,steer:dS,thr:dT,x0:M.x0,u0:M.u0,Ad:M.Ad,Bd:M.Bd,r2:M.r2};
  }finally{E.mjSwap(job,false);}
  // the physical core at the hold's own radius and slip
  const [bm,rm,vm,em]=rec.mean,rho=R+em,vx=vm*Math.cos(bm),vy=vm*Math.sin(bm),r=vm/rho,w=E.WHEELS.map(Wh=>1.1*Math.hypot(vx-r*Wh.ry,vy+r*Wh.rx)/CAR.Rw);
  const g=[vm,rec.mean[5],rec.mean[7]];const q=P.equilibrium(rho,R,bm,rec.gi,g,true);
  rec.eq={ok:q.fn<1e-6,v:q.v,delta:q.delta,p:q.p,thr:q.thr,rpm:q.rpm,rho,beta:bm};if(rec.eq.ok){const L=P.linearise(q);rec.A=L.A;rec.B=L.B;rec.C=L.C;const Ls=P.linearise(q,true);rec.As=Ls.A;rec.Bs=Ls.B;rec.Cs=Ls.C;rec.f0s=Ls.f0;}   // As/Bs/Cs: with the steering (inputs p, thr; outputs β r v δ)
  out.push(rec);const f=(x,dd=1)=>(+x).toFixed(dd);
  console.log(`${String(R).padStart(3)}/${rec.b} t${ti} hold β ${f(bm*57.3)}±${f(rec.sd[0]*57.3)} ρ ${f(rho)} | v ${f(vm*3.6)} → ${f(q.v*3.6)} | δ ${f(rec.mean[5]*57.3)} → ${f(q.delta*57.3)} | p ${f(rec.mean[6]*57.3)} → ${f(q.p*57.3)} | thr ${f(rec.mean[7],2)} → ${f(q.thr,2)} | table op β ${f(M.x0[0]*57.3)} δ ${f(M.x0[5]*57.3)} thr ${f(M.u0[1],2)}${rec.eq.ok?'':' (model: no steady slide)'}`);}
fs.writeFileSync(process.argv[2],JSON.stringify({drive,pts:out}));
