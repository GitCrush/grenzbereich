// Why does a computed table point fail its validation? (decision 11 Oct) The validation's hold (dcValidGen: the
// table's regulator, its own steady slide as reference, 6 s on the ring R) repeated with the road rough or smooth and
// the ruts on or off; per 0.5 s the slip, the radial error, the steering and throttle.
// node hybrid/valdiag.js R b   env: DRIVE, V=1 (trace)
const {loadEngine}=require('../load');const f=(x,d=1)=>(+x).toFixed(d);const R=+process.argv[2],bd=+process.argv[3],V=process.env.V==='1';
for(const [amp,rut] of [[1,1],[0,1],[1,0],[0,0]]){const E=loadEngine();const drive=process.env.DRIVE||'awd';E.cfg.drive=drive;E.applyDriveSetup(drive);E.selectTrack('rundkurs');
  if(!rut)E.ROAD[E.cfg.surface].rut=0;if(process.env.SETA)Object.assign(E.AP,JSON.parse(process.env.SETA));const P=E.phSetup();
  // the grid point from its neighbours as the table computes it: chain 75/22 → here
  let M=E.phPoint(P,75,22/57.3,[0.92*Math.sqrt(P.sur.mu*9.81*75),drive==='rwd'?-0.29:0.06,0.5]);
  for(const [Rk,bk] of [[R,22],[R,bd]]){const s=M.q;M=E.phPoint(P,Rk,bk/57.3,[s.v*Math.sqrt(Rk/s.R||Rk/M.R),s.delta,s.thr]);}
  if(!M.ok){console.log('no usable slide');process.exit();}const ch=E.dcChain();M.K=E.dcGain(M,ch);
  const TR=E.buildTrack({name:'ring',type:'seg',segs:[{R,a:360}],w:30,ex:null});TR.patches=[];TR.patchAt=new Array(TR.N);
  const job={S:E.cloneState(E.S),IN:{steer:0,thr:0,brk:0,hb:0},VERT:E.cloneState(E.VERT),TRACK:TR};E.mjSwap(job,true);
  try{const S=E.S;E.VERT.amp=amp;let cx=0,cy=0;for(const p of TR.pts){cx+=p.x;cy+=p.y;}cx/=TR.N;cy/=TR.N;const ks=TR.pts[10].ks||1,d=-ks;S.noLead=true;
    {const p0=TR.pts[10],v0=M.x0[2],bs=-d*M.x0[0];S.x=p0.x;S.y=p0.y;S.psi=Math.atan2(p0.ty,p0.tx)-bs;S.idx=10;S.vx=v0*Math.cos(bs);S.vy=v0*Math.sin(bs);S.r=-d*v0/R;
     S.delta=-d*M.x0[5];S.dDelta=0;S.hand=S.delta*E.CAR.ratio;S.dq=null;const w=S.vx/E.CAR.Rw;S.w=[w,w,w,w];S.gi=E.dcGearOf(v0);S.rpm=Math.max(1200,w*E.CAR.gears[S.gi].r*E.CAR.final*9.55);S.we=S.rpm*0.10472;
     if(M.q&&M.q.w){const sw=-d>0?[0,1,2,3]:[1,0,3,2];S.w=sw.map(k=>M.q.w[k]);S.gi=M.q.gi;const dr=drive==='awd'?[0,1,2,3]:drive==='fwd'?[0,1]:[2,3];S.rpm=dr.reduce((a,k)=>a+S.w[k],0)/dr.length*E.CAR.gears[S.gi].r*E.CAR.final*9.5493;S.we=S.rpm*0.10472;S.boost=M.x0[7];}   // as dcValidGen
     E.vertInit();}
    const C1=E.dcCtl(M,ch,M.K);let t=0,k=0,c1=S.delta,t1=M.u0[1],s2=0,n=0,emax=0,line='';const DT=E.DT;
    while(t<6){const S=E.S,v=Math.hypot(S.vx,S.vy),be=Math.atan2(S.vy,Math.abs(S.vx)),e=Math.hypot(S.x-cx,S.y-cy)-R,chi=S.psi+Math.atan2(S.vy,S.vx),th=Math.atan2(S.y-cy,S.x-cx)+ks*Math.PI/2;let h=chi-th;h=Math.atan2(Math.sin(h),Math.cos(h));
      const x=[-d*be,-d*S.r,v,e,-d*h,-d*S.delta];if(M.Ad.length>6)x.push(...E.dcXh());if(k%6===0){const u=C1.step(x,-d*(S.dLP===undefined?S.delta:S.dLP),{b:M.x0[0],e:0},null,null);c1=-d*u[0];t1=u[1];}
      if(k%180===0)line+=` ${f(t,1)}s:β${f(-x[0]*57.3,0)} e${f(e,1)} δ${f(x[5]*57.3,0)} thr${f(t1,2)} g${S.gi}`;
      E.IN.steer=Math.max(-1,Math.min(1,c1/E.CAR.maxSteer));E.IN.thr=t1;E.IN.brk=0;E.IN.hb=0;E.step(DT);t+=DT;k++;
      if(Math.abs(x[0])>1.2||Math.abs(x[3])>28){line+=' LOST';break;}if(t>1.5){const eb=x[0]-M.x0[0];s2+=eb*eb;n++;emax=Math.max(emax,Math.abs(e));}}
    const rms=Math.sqrt(s2/Math.max(1,n))*57.3;console.log(`${drive} R${R}/${bd} road ${amp?'rough':'smooth'} ruts ${rut?'on':'off'}: slip rms ${f(rms,1)}° (bar 3) |e| max ${f(emax,1)} m (bar 4) → ${n>0&&rms<3&&emax<4?'HELD':'fails'} | op v ${f(M.x0[2]*3.6,0)} δ ${f(M.x0[5]*57.3,1)} thr ${f(M.u0[1],2)}`);if(V)console.log('   ',line);
  }finally{E.mjSwap(job,false);}}
