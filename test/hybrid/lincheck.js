// Is the regulator designed on the 8-state controller model stable on the full linearised core? (decision 11 Oct)
// Linear core (10 states, 60 Hz) + the path's kinematics (e, h, linearised) + the driver's chain (delay, lag), closed by
// dcCtl with the table's gain; 6 s from β + 1°. Beside it the same loop on the controller's own model.
// node hybrid/lincheck.js R b   env: DRIVE, SETA
const {loadEngine}=require('../load');const E=loadEngine();const drive=process.env.DRIVE||'awd';E.cfg.drive=drive;E.applyDriveSetup(drive);E.selectTrack('rundkurs');
if(process.env.SETA)Object.assign(E.AP,JSON.parse(process.env.SETA));
const P=E.phSetup(),R=+process.argv[2],bd=+process.argv[3],b=bd/57.3,f=(x,d=2)=>(+x).toFixed(d);
let M=E.phPoint(P,75,22/57.3,[0.92*Math.sqrt(P.sur.mu*9.81*75),drive==='rwd'?-0.29:0.06,0.5]);for(const [Rk,bk] of [[R,22],[R,bd]]){const s=M.q;M=E.phPoint(P,Rk,bk/57.3,[s.v*Math.sqrt(Rk/M.R),s.delta,s.thr]);}
const ch=E.dcChain();M.K=E.dcGain(M,ch);const q=M.q,L=E.phLin(P,q),n=10,dt=1/60;
const Mx=Array.from({length:n+2},()=>new Array(n+2).fill(0));for(let i=0;i<n;i++){for(let j=0;j<n;j++)Mx[i][j]=L.A[i][j]*dt;Mx[i][n]=L.B[i][0]*dt;Mx[i][n+1]=L.B[i][1]*dt;}
const Ex=E.phExpm(Mx),Phi=Ex.slice(0,n).map(r=>r.slice(0,n)),Gam=Ex.slice(0,n).map(r=>r.slice(n));
const drv=P.drv,Cs=new Array(n).fill(0);for(const i of drv)Cs[3+i]=E.CAR.Rw/drv.length;for(let j=0;j<n;j++)Cs[j]-=L.C[2][j];
const yOf=x=>[L.C[0],L.C[1],L.C[2],Cs,[0,0,0,0,0,0,0,1,0,0],L.C[3]].map(c=>c.reduce((a,v,j)=>a+v*x[j],0));   // β r v s b δ deviations
function run(core){const C1=E.dcCtl(M,ch,M.K);let x=new Array(n).fill(0),xm=new Array(8).fill(0),e=0,h=0,lag=M.u0[0];const Q=new Array(ch.nd).fill(M.u0[0]);let thr=M.u0[1],out=[],mx=0;
  // β + 1° (the core: vy changed so that β grows by 1°)
  if(core){const v=q.v,bb=q.beta;x[0]=-v*Math.sin(bb)*(-1/57.3)*0+(v*Math.cos(bb+(-1/57.3))-v*Math.cos(bb));x[1]=v*Math.sin(bb-1/57.3)-v*Math.sin(bb);}else xm[0]=-1/57.3;
  for(let k=0;k<360;k++){const y=core?yOf(x):[xm[0],xm[1],xm[2],xm[6],xm[7],xm[5]];const X=[M.x0[0]+y[0],M.x0[1]+y[1],M.x0[2]+y[2],e,h,M.x0[5]+y[5],M.x0[6]+y[3],M.x0[7]+y[4]];
    const u=C1.step(X,lag,{b:M.x0[0],e:0},null,null);Q.unshift(u[0]);const pd=Q.pop();lag=lag+(pd-lag)*(1-ch.a);thr=u[1];const du=[lag-M.u0[0],thr-M.u0[1]];
    if(core){x=Phi.map((r,i)=>r.reduce((a,v,j)=>a+v*x[j],0)+Gam[i][0]*du[0]+Gam[i][1]*du[1]);}
    else{const xx=X.map((v,i)=>v);const nx=M.Ad.map((r,i)=>r.reduce((a,v,j)=>a+v*xx[j],0)+M.Bd[i][0]*lag+M.Bd[i][1]*thr+M.cd[i]);xm=nx.map((v,i)=>v-M.x0[i]);e=nx[3];h=nx[4];continue;}
    // the path for the core: e' = −v sin h, h' = r + β' − v cos h/(R+e) around the steady slide (h0 = 0, e0 = 0)
    const yn=yOf(x),v=q.v+yn[2],rr=q.v/q.rho+yn[1];const db=(yn[0]-y[0])*60;e+=dt*(-v*Math.sin(h));h+=dt*(rr+db-v*Math.cos(h)/(R+e));
    mx=Math.max(mx,Math.abs(y[0]));if(k%60===59)out.push(f(y[0]*57.3,1));}
  return out.join(' ');}
console.log(`${drive} R${R}/${bd}: β deviation (°) each second | on the controller's model: ${run(false)}`);
console.log(`${' '.repeat(drive.length+R.toString().length+bd.toString().length+3)}| on the full core:          ${run(true)}`);
