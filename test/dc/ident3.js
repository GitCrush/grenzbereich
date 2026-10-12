// Identify the drift on a ring with the steering in the state (NS 7: [β r v e h δ δ̇], inputs [p = S.dLP, thr]) at the driver's
// reaction REACT, held by a bootstrap controller (MODEL0, a neighbouring point's model), then compute the augmented LQI (KA).
// The lead filter of the driver model is off (the controller anticipates itself). References for the bootstrap: β target, e 0,
// h 0, r = v/R, v and δ washed out (their 2 s means) so a neighbour's equilibrium values do not bias the new point.
// env: R, BT, DRIVE, SURF, REACT, MODEL0, TEX, TEND, AMPD, AMPT, Q, Rw, QI, OUT, TRACE, SEED
const L=require('./dclib'),CT=require('./ctl');const {f}=L;const env=process.env;
const R=+(env.R||83),BT=+(env.BT||20)/57.3,react=+(env.REACT||0);
const NS=+(env.NS||6);const o=L.setup({smooth:env.SMOOTH==='1',R,drive:env.DRIVE,surf:env.SURF,NS,react});const {E,S,C,d}=o,DT=E.DT;C.drvLead=0;
const en=E.measureEnvelope();const q=E.driftEqAt(en,R,BT);const TFF=q?q.thr*Math.min(1,Math.max(0.3,R/30)):0.6;
const TEX=+(env.TEX||6),TEND=+(env.TEND||40),AMPD=+(env.AMPD||1.5)/57.3,AMPT=+(env.AMPT||0.08);
const M0=JSON.parse(require('fs').readFileSync(env.MODEL0));const K0=CT.mk(M0,C,DT);
let t=0,k=0,lost=false,cmd=0,thr=0,vL=null,dL=null,ex=0,eT=0;const X=[],U=[],DUMP=[];let seed=+(env.SEED||7);const rnd=()=>{seed=(seed*16807)%2147483647;return seed/2147483647;};
while(t<TEND){const x=L.obs(o);
  if(k%6===0){const kC=L.aimK(o),thf=L.thF(o);
    const TB=+(env.TB||1);   // the bootstrap controller takes over at TB (it initiates the slide itself; the old run-up spun the RWD car on rough roads)
    if(t<Math.min(1,TB)){cmd=thf+0.3*(x[2]*kC-S.r);thr=0.2;}else if(t<TB){cmd=thf+0.3*(x[2]*kC-S.r);thr=TFF;}
    else{if(vL===null){vL=x[2];dL=S.delta;}vL+=(x[2]-vL)/120;dL+=(S.delta-dL)/120;
      if(t>TEX){if(k%(+(env.EXD||54))===0)ex=(rnd()-0.5)*2*AMPD;if(k%(+(env.EXT||90))===0)eT=(rnd()-0.5)*2*AMPT;}
      [cmd,thr]=K0.step(M0.Ad.length===NS?x:L.obs(o,M0.Ad.length),S.dLP,{b:d*BT,e:0,h:0,r:Math.sign(M0.x0[1])*x[2]/R,v:vL,d:dL},t>TEX?[ex,eT]:null);}
    if(t>TEX+1){X.push([...x,S.dLP,thr]);if(env.DUMP)DUMP.push([...x.slice(0,6),S.dLP,thr,S.dDelta,S.hand/C.ratio,...S.w.map(w=>w*C.Rw),S.we,...S.Fz,S.dComp,S.ax,S.ay,cmd]);}
    if(env.TRACE&&k%(+env.TRACE)===0)console.log(f(t,1),'β',f(x[0]*57.3,1),'r',f(x[1]*57.3,1),'v',f(x[2]*3.6,0),'e',f(x[3],1),'h',f(x[4]*57.3,1),'δ',f(S.delta*57.3,1),'cmd',f(cmd*57.3,1),'thr',f(thr));}
  E.IN.steer=Math.max(-1,Math.min(1,cmd/C.maxSteer));E.IN.thr=thr;E.IN.brk=0;E.IN.hb=0;E.step(DT);t+=DT;k++;
  if(Math.abs(x[0])>1.2||Math.abs(x[3])>28){lost=true;console.log('ident run lost at',f(t));break;}}
// pairs (x_k, u_k) → x_{k+1}: X rows hold [x(7), p, thr] at frame starts
const Xs=X.map(r=>r.slice(0,NS)),Us=X.map(r=>[r[NS],r[NS+1]]);const M=(env.FREE==='1'?L.fit:L.fitM)(Xs,Us,+(env.RIDGE||1e-9));const nm=['β','r','v','e','h','δ','δ̇'];
const ch=L.chain(C,DT);const Qw=JSON.parse(env.Q||'[1,0.05,0.2,1,1,0,0]'),Rw=JSON.parse(env.Rw||'[10,10]'),QI=JSON.parse(env.QI||'[0.1,0.05]');
M.KA=L.lqiAug(M,Qw,Rw,QI,ch.nd,ch.a);M.Qw=Qw;M.Rw=Rw;M.QI=QI;M.nd=ch.nd;M.a=ch.a;M.R=R;M.BT=BT;M.Tff=TFF;M.drive=E.cfg.drive;M.react=react;M.lost=lost;
console.log(`[${E.cfg.drive} R${R} β${f(BT*57.3,0)} r${react} nd${ch.nd}] n ${X.length} | R² `+M.r2.map((v,i)=>nm[i]+' '+f(v)).join(' ')+` | x0 β ${f(M.x0[0]*57.3,1)} v ${f(M.x0[2]*3.6,0)} e ${f(M.x0[3],1)}±${f(Math.sqrt(Xs.reduce((a,x)=>a+(x[3]-M.x0[3])**2,0)/Xs.length),1)} | u0 p ${f(M.u0[0]*57.3,1)} thr ${f(M.u0[1])}`+(lost?' LOST':''));
if(env.DUMP)require('fs').writeFileSync(env.DUMP,JSON.stringify(DUMP));require('fs').writeFileSync(env.OUT||__dirname+'/model.json',JSON.stringify(M));
