// How robust is the drift controller's gain to the model it was designed on? (decision 10 Oct: in the game the same
// ring held in one build and was lost in the next after a change that should not matter – R 55 m failed, R 83 m held,
// then the other way round.) Identifies one point several times with other excitation sequences (AP.dcSeedOff) and
// prints the closed-loop spectral radius of every gain on every model (per 1/60 s frame; < 1 stable).
// node dc/robust.js R b [runs]   env: DRIVE, REACT, RW (JSON input weights), OUT (JSON file: the models and the chain, for
//   dc/robust.py), PTS="R:b,R:b" (several points instead of R b), COND=n,r,o (conditions: n the identification's own –
//   smooth road, on the band; r the rough road; o held 3.5 m beside the band, in the loose gravel)
const {loadEngine}=require('../load');const f=(x,d=3)=>(+x).toFixed(d);
const R=+(process.argv[2]||55),bd=+(process.argv[3]||22),runs=+(process.argv[4]||4);
const E=loadEngine();if(process.env.DRIVE){E.cfg.drive=process.env.DRIVE;E.applyDriveSetup(E.cfg.drive);}if(process.env.REACT)E.applyReaction(+process.env.REACT);
E.AP.drift=1;E.AP.dcUse=true;E.selectTrack('rundkurs');E.AP.on=true;E.apPrepare();const T=E.AP.dcTab,ch=E.dcChain();
const near=(R0,b0)=>{let best=null,bd0=1e9;for(const m of T.pts){if(!m.ok)continue;const dd=Math.abs(Math.log(m.R/R0))+Math.abs(m.b-b0)*3;if(dd<bd0&&!(m.R===R0&&Math.abs(m.b-b0)<1e-6)){bd0=dd;best=m;}}return best;};

const base={lat:E.AP.env.lat,__base:{S:JSON.parse(JSON.stringify(E.S)),VERT:JSON.parse(JSON.stringify(E.VERT||{}))}};
// inside the measurement's world as the game runs it (mjSwap: no ideal line – the band follows the ring's centre –, uniform
// surroundings, no height profile)
function ident(R,bd,M0,off){E.AP.dcSeedOff=off;const job={S:E.cloneState(E.S),IN:{steer:0,thr:0,brk:0,hb:0},VERT:E.cloneState(E.VERT),TRACK:E.TRACK};E.mjSwap(job,true);
  try{const g=E.dcIdentGen(R,bd/57.3,M0,base,()=>false);let r=g.next();while(!r.done)r=g.next();return r.value;}finally{E.mjSwap(job,false);}}
const out={ch,Q:E.DC_Q,QI:E.DC_QI,RW:E.DC_RW,SC:E.DC_SC,SU:E.DC_SU,pts:[]};
const pts=process.env.PTS?process.env.PTS.split(',').map(x=>x.split(':').map(Number)):[[R,bd]];
for(const [R,bd] of pts){const M0=near(R,bd/57.3);console.log('point R',R,'b',bd,'| chain nd',ch.nd,'a',f(ch.a),'| start model R',M0.R,'b',f(M0.b*57.3,0));
const Ms=[];const tabM=T.pts.find(m=>m.R===R&&Math.round(m.b*57.3)===bd);const conds=(process.env.COND||'n').split(',');for(const cd of conds)for(let k=0;k<runs;k++){E.AP.dcIdRough=cd==='r';E.AP.dcIdE=cd==='o'?3.5:0;const M=ident(R,bd,M0,k*1000);E.AP.dcIdRough=false;E.AP.dcIdE=0;if(!M){console.log('run',cd,k,'lost');continue;}M.cond=cd;Ms.push(M);
  console.log('run',M.cond,k,'ok',M.ok,'| x0 β',f(M.x0[0]*57.3,1),'r',f(M.x0[1]*57.3,1),'v',f(M.x0[2]*3.6,1),'e',f(M.x0[3],2),'δ',f(M.x0[5]*57.3,1),'| u0 p',f(M.u0[0]*57.3,1),'thr',f(M.u0[1],2),'| R²',M.r2.map(v=>f(v,2)).join(' '));}
const Rw=process.env.RW?JSON.parse(process.env.RW):null;
function aug(M){const n=6,nd=ch.nd,a=ch.a,N=n+1+nd+2,dtk=1/60,A=Array.from({length:N},()=>new Array(N).fill(0)),B=Array.from({length:N},()=>[0,0]);
  for(let i=0;i<n;i++){for(let j=0;j<n;j++)A[i][j]=M.Ad[i][j];A[i][n]=M.Bd[i][0];B[i][1]=M.Bd[i][1];}
  A[n][n]=a;if(nd>0)A[n][n+nd]=1-a;else B[n][0]=1-a;for(let j=0;j<nd;j++){if(j===0)B[n+1][0]=1;else A[n+1+j][n+j]=1;}
  A[N-2][0]=dtk;A[N-2][N-2]=1;A[N-1][3]=dtk;A[N-1][N-1]=1;return {A,B,N};}
const mm=(A,B)=>A.map(r=>B[0].map((_,j)=>{let s=0;for(let k=0;k<r.length;k++)s+=r[k]*B[k][j];return s;}));
function rho(A){let M=A.map(r=>r.slice()),lg=0,k=1;for(let it=0;it<12;it++){M=mm(M,M);k*=2;let nr=0;for(const r of M)for(const v of r)nr=Math.max(nr,Math.abs(v));if(!(nr>0))return 0;lg=2*lg+Math.log(nr);M=M.map(r=>r.map(v=>v/nr));}return Math.exp(lg/k);}
const Ks=Ms.map(M=>E.dcGain(M,ch,Rw||undefined));
console.log('closed-loop spectral radius: rows gain of run i, columns model of run j');
for(let i=0;i<Ms.length;i++){const row=[];for(let j=0;j<Ms.length;j++){const {A,B,N}=aug(Ms[j]),K=Ks[i];const Acl=A.map((r,a)=>r.map((v,b)=>v-B[a][0]*K[0][b]-B[a][1]*K[1][b]));row.push(f(rho(Acl),4));}console.log('  gain',i,row.join('  '));}
// open loop (vehicle part) for reference
console.log('open-loop vehicle spectral radius per run:',Ms.map(M=>f(rho(M.Ad),4)).join(' '));
out.pts.push({R,b:bd,tab:tabM?{Ad:tabM.Ad,Bd:tabM.Bd,x0:tabM.x0,u0:tabM.u0,ok:tabM.ok}:null,runs:Ms.map(M=>({Ad:M.Ad,Bd:M.Bd,cd:M.cd,x0:M.x0,u0:M.u0,ok:M.ok,r2:M.r2,cond:M.cond}))});}
if(process.env.OUT)require('fs').writeFileSync(process.env.OUT,JSON.stringify(out));
