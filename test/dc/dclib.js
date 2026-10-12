// Shared helpers for the drift-controller experiments: ring setup, state observation, least squares, discrete LQR.
// State x (NS 5): [β, r, v, e (radial error, + outward), h (course − ring tangent)]; NS 7 adds the steering [δ road wheel, δ̇]
// (the input is then the hands' target after the driver's delay and lag, S.dLP, and the controller is augmented with that chain).
const {loadEngine}=require('../load');
const f=(x,d=2)=>(+x).toFixed(d);
function setup(o){const R=o.R,react=o.react||0;const E=loadEngine();E.applyReaction(react);E.cfg.surface=o.surf||'gravel';E.cfg.drive=o.drive||'awd';E.applyDriveSetup(E.cfg.drive);
  E.TRACKS.ringX={name:'ring',type:'seg',segs:[{R,a:360}],w:30,ex:null};E.selectTrack('ringX');E.cfg.uniform=true;E.cfg.profile=false;E.AP.on=true;
  const S=E.S,T=E.TRACK,C=E.CAR;let cx=0,cy=0;for(const p of T.pts){cx+=p.x;cy+=p.y;}cx/=T.N;cy/=T.N;
  const p0=T.pts[10];S.x=p0.x;S.y=p0.y;S.psi=Math.atan2(p0.ty,p0.tx);S.idx=10;const vS=o.v0||Math.sqrt(0.75*0.86*9.81*R);S.vx=vS;S.vy=0;S.r=0;const w=vS/C.Rw;S.w=[w,w,w,w];
  S.gi=Math.max(2,Math.min(C.gears.length-1,2+Math.floor(vS/9)));S.rpm=Math.max(1200,w*C.gears[S.gi].r*C.final*9.55);S.we=S.rpm*0.10472;E.vertInit();
  if(o.smooth)E.VERT.amp=0;   // no road roughness (identification on a clean signal; the bumps are an unpredictable disturbance)
  const ks=T.pts[10].ks||1;return {E,S,C,R,cx,cy,ks,d:-(Math.sign(ks)||1),NS:o.NS||5};}
function obs(o,nsO){const {S,C,cx,cy,ks,R}=o;const NS=nsO||o.NS;const v=Math.hypot(S.vx,S.vy),beta=Math.atan2(S.vy,Math.abs(S.vx)),e=Math.hypot(S.x-cx,S.y-cy)-R;
  const chi=S.psi+Math.atan2(S.vy,S.vx),th=Math.atan2(S.y-cy,S.x-cx)+ks*Math.PI/2;let h=chi-th;h=Math.atan2(Math.sin(h),Math.cos(h));
  const x=[beta,S.r,v,e,h];if(NS>=6)x.push(S.delta);if(NS===7)x.push(S.dDelta||0);
  if(NS===12){const V=o.E.VERT;x.push(V.z,V.dz,V.th,V.dth,V.ph,V.dph);}   // the body (heave, pitch, roll and their rates): the slow part of the load transfer, without the wheel hop
  if(NS===11||NS===10){const F0=o.C.m*9.81/4;for(let i=0;i<4;i++)x.push(S.Fz[i]/F0-1);if(NS===11)x.push(S.dComp);}   // wheel loads (body roll/pitch/heave) and steering compliance
  return x;}
// pure-pursuit curvature towards the ring point Ld ahead (the weak path term of the establishing law)
function aimK(o){const {S,cx,cy,ks,R}=o;const Ld=Math.min(12,0.4*R),a0=Math.atan2(S.y-cy,S.x-cx)+Ld/R*ks,tx=cx+R*Math.cos(a0),ty=cy+R*Math.sin(a0);
  const vDir=S.psi+Math.atan2(S.vy,S.vx);let al=Math.atan2(ty-S.y,tx-S.x)-vDir;al=Math.atan2(Math.sin(al),Math.cos(al));return 2*Math.sin(al)/Ld;}
const thF=o=>Math.atan2(o.S.vy+o.C.a*o.S.r,o.S.vx);
function lsq(Phi,Y,ridge=1e-9){const n=Phi[0].length,m=Y[0].length;const A=Array.from({length:n},()=>new Float64Array(n)),B=Array.from({length:n},()=>new Float64Array(m));
  for(let i=0;i<Phi.length;i++){for(let a=0;a<n;a++){for(let b=0;b<n;b++)A[a][b]+=Phi[i][a]*Phi[i][b];for(let c=0;c<m;c++)B[a][c]+=Phi[i][a]*Y[i][c];}}
  for(let a=0;a<n;a++)A[a][a]+=ridge;const M=A.map((r,i)=>[...r,...B[i]]);
  for(let c=0;c<n;c++){let p=c;for(let r=c+1;r<n;r++)if(Math.abs(M[r][c])>Math.abs(M[p][c]))p=r;[M[c],M[p]]=[M[p],M[c]];const pv=M[c][c];for(let j=c;j<n+m;j++)M[c][j]/=pv;for(let r=0;r<n;r++)if(r!==c){const fc=M[r][c];for(let j=c;j<n+m;j++)M[r][j]-=fc*M[c][j];}}
  return M.map(r=>r.slice(n));}
const mm=(A,B)=>A.map(r=>B[0].map((_,j)=>r.reduce((s,v,k)=>s+v*B[k][j],0))),tr=A=>A[0].map((_,j)=>A.map(r=>r[j])),add=(A,B)=>A.map((r,i)=>r.map((v,j)=>v+B[i][j])),sub=(A,B)=>A.map((r,i)=>r.map((v,j)=>v-B[i][j]));
function inv(A){const n=A.length;const M=A.map((r,i)=>[...r,...A.map((_,j)=>i===j?1:0)]);for(let c=0;c<n;c++){let p=c;for(let r=c+1;r<n;r++)if(Math.abs(M[r][c])>Math.abs(M[p][c]))p=r;[M[c],M[p]]=[M[p],M[c]];const pv=M[c][c];for(let j=0;j<2*n;j++)M[c][j]/=pv;for(let r=0;r<n;r++)if(r!==c){const fc=M[r][c];for(let j=0;j<2*n;j++)M[r][j]-=fc*M[c][j];}}return M.map(r=>r.slice(n));}
function dlqr(A,B,Q,Rm){let P=Q.map(r=>r.slice());for(let it=0;it<20000;it++){const BtP=mm(tr(B),P),K=mm(inv(add(Rm,mm(BtP,B))),mm(BtP,A));const Pn=add(Q,mm(tr(A),mm(P,sub(A,mm(B,K)))));let dlt=0,sc=0;for(let i=0;i<P.length;i++)for(let j=0;j<P.length;j++){dlt=Math.max(dlt,Math.abs(Pn[i][j]-P[i][j]));sc=Math.max(sc,Math.abs(Pn[i][j]));}P=Pn;if(dlt<1e-9*Math.max(1,sc))break;}
  const BtP=mm(tr(B),P);return mm(inv(add(Rm,mm(BtP,B))),mm(BtP,A));}
// fit x_{k+1} = Ad x_k + Bd u_k + c ; returns model and 1-step R² of the increments
function fit(X,U,ridge){const N=X.length-1,n=X[0].length,m=U[0].length;const Phi=[],Y=[];for(let i=0;i<N;i++){Phi.push([...X[i],...U[i],1]);Y.push(X[i+1]);}
  const W=lsq(Phi,Y,ridge);const Ad=[...Array(n).keys()].map(i=>[...Array(n).keys()].map(j=>W[j][i])),Bd=[...Array(n).keys()].map(i=>[...Array(m).keys()].map(j=>W[n+j][i])),cd=[...Array(n).keys()].map(i=>W[n+m][i]);
  const r2=[...Array(n).keys()].map(s=>{let ss=0,st=0,mu=0;for(let i=0;i<N;i++)mu+=Y[i][s]-X[i][s];mu/=N;for(let i=0;i<N;i++){const pr=Ad[s].reduce((a,v,j)=>a+v*X[i][j],0)+Bd[s].reduce((a,v,j)=>a+v*U[i][j],0)+cd[s];ss+=(Y[i][s]-pr)**2;st+=(Y[i][s]-X[i][s]-mu)**2;}return 1-ss/st;});
  const x0=[...Array(n).keys()].map(j=>X.reduce((a,x)=>a+x[j],0)/X.length),u0=[...Array(m).keys()].map(j=>U.reduce((a,u)=>a+u[j],0)/U.length);return {Ad,Bd,cd,x0,u0,r2};}
// LQI: weights per state in physical scales, plus integrators of β and e (dt 1/60 s)
const SC=[1/57.3,5/57.3,1,0.5,1/57.3,2/57.3,0.1,0.1,0.1,0.1,1/57.3,1],SU=[2/57.3,0.1];
function lqi(M,Qw,Rw,QI){const n=M.Ad.length,dtk=1/60;const Aa=M.Ad.map(r=>[...r,0,0]);const ib=Array(n+2).fill(0);ib[0]=dtk;ib[n]=1;const ie=Array(n+2).fill(0);ie[3]=dtk;ie[n+1]=1;Aa.push(ib,ie);
  const Ba=M.Bd.map(r=>r.slice());Ba.push([0,0],[0,0]);const q=[...Qw.slice(0,n),...QI],s=[...SC.slice(0,n),1/57.3,0.5];const Qa=q.map((qq,i)=>q.map((_,j)=>i===j?qq/s[i]**2:0)),Rm=Rw.map((qq,i)=>Rw.map((_,j)=>i===j?qq/SU[i]**2:0));
  return dlqr(Aa,Ba,Qa,Rm);}
// Augmented LQI for NS 7 models (input 0 = p, the lagged hand target): the driver chain cmd → delay nd frames → lag (p_{k+1} = a p_k + (1−a) cmd_{k−nd})
// is known exactly, so it becomes part of the state: z = [x (7), p, buffer cmd_{k−1..k−nd}, ∫β, ∫e]; inputs [cmd_k, thr_k].
function lqiAug(M,Qw,Rw,QI,nd,a){const n=M.Ad.length,N=n+1+nd+2,dtk=1/60;const A=Array.from({length:N},()=>Array(N).fill(0)),B=Array.from({length:N},()=>[0,0]);
  for(let i=0;i<n;i++){for(let j=0;j<n;j++)A[i][j]=M.Ad[i][j];A[i][n]=M.Bd[i][0];B[i][1]=M.Bd[i][1];}
  A[n][n]=a;if(nd>0)A[n][n+nd]=1-a;else B[n][0]=1-a;
  for(let j=0;j<nd;j++){if(j===0)B[n+1][0]=1;else A[n+1+j][n+j]=1;}
  A[N-2][0]=dtk;A[N-2][N-2]=1;A[N-1][3]=dtk;A[N-1][N-1]=1;
  const qd=Array(N).fill(0);for(let i=0;i<n;i++)qd[i]=(Qw[i]||0)/SC[i]**2;qd[N-2]=QI[0]/(1/57.3)**2;qd[N-1]=QI[1]/0.5**2;
  const Q=qd.map((q,i)=>qd.map((_,j)=>i===j?q:0)),Rm=Rw.map((q,i)=>Rw.map((_,j)=>i===j?q/SU[i]**2:0));return dlqr(A,B,Q,Rm);}
// the driver chain of the engine at the current reaction: delay in frames (60 Hz) and the lag's per-frame factor
const chain=(C,DT)=>({nd:Math.round(Math.max(1,Math.round(C.drvDelay/DT))*DT*60),a:Math.pow(1-Math.min(1,DT/C.drvLag),Math.round(1/60/DT))});
// Structured fit: each state's row uses only the regressors its physics allows (closed-loop data make the free fit pick up
// spurious couplings, e.g. the yaw rate 'depending' on the ring position). Regressor order: [x (n), u (m), 1].
// n 5 (u = δ, thr) / n 6 (x adds δ, u = p, thr):  β, r, v ← β r v δ thr ;  e ← v e h ;  h ← all states and inputs ;  δ ← β r v δ p.
function masks(n){if(n===12){const all=[...Array(15).keys()];const dyn=all.filter(c=>c!==3&&c!==4&&c!==12);   // 12 = p
    return [dyn,dyn,dyn,[2,3,4,14],all.filter(c=>c!==12),[0,1,2,5,12,14],dyn,dyn,dyn,dyn,dyn,dyn];}
  if(n===10){const all=[...Array(13).keys()];const dyn=all.filter(c=>c!==3&&c!==4&&c!==10);   // 10 = p
    return [dyn,dyn,dyn,[2,3,4,12],all.filter(c=>c!==10),[0,1,2,5,10,12],dyn,dyn,dyn,dyn];}
  if(n===11){const all=[...Array(14).keys()];const dyn=all.filter(c=>c!==3&&c!==4&&c!==11);   // 11 = p
    return [dyn,dyn,dyn,[2,3,4,13],all.filter(c=>c!==11),[0,1,2,5,10,11,13],dyn,dyn,dyn,dyn,dyn];}const I=(...a)=>a;if(n===5)return [I(0,1,2,5,6,7),I(0,1,2,5,6,7),I(0,1,2,5,6,7),I(2,3,4,7),I(0,1,2,3,4,5,6,7)];
  return [I(0,1,2,5,7,8),I(0,1,2,5,7,8),I(0,1,2,5,7,8),I(2,3,4,8),I(0,1,2,3,4,5,7,8),I(0,1,2,5,6,8)];}
function fitM(X,U,ridge=1e-9){const N=X.length-1,n=X[0].length,m=U[0].length,mk=masks(n);const Ad=[],Bd=[],cd=[],r2=[];
  for(let s=0;s<n;s++){const cols=mk[s];const Phi=[],Y=[];for(let i=0;i<N;i++){const full=[...X[i],...U[i],1];Phi.push(cols.map(c=>full[c]));Y.push([X[i+1][s]]);}
    const W=lsq(Phi,Y,ridge).map(r=>r[0]);const row=Array(n+m+1).fill(0);cols.forEach((c,j)=>row[c]=W[j]);Ad.push(row.slice(0,n));Bd.push(row.slice(n,n+m));cd.push(row[n+m]);
    let ss=0,st=0,mu=0;for(let i=0;i<N;i++)mu+=Y[i][0]-X[i][s];mu/=N;for(let i=0;i<N;i++){const pr=cols.reduce((a,c,j)=>a+W[j]*Phi[i][j],0);ss+=(Y[i][0]-pr)**2;st+=(Y[i][0]-X[i][s]-mu)**2;}r2.push(1-ss/st);}
  const x0=[...Array(n).keys()].map(j=>X.reduce((a,x)=>a+x[j],0)/X.length),u0=[...Array(m).keys()].map(j=>U.reduce((a,u)=>a+u[j],0)/U.length);return {Ad,Bd,cd,x0,u0,r2};}
// Model-consistent steady state for a target β (and e = 0): solve (I − Ad) x − Bd u = c with x[0] = β*, x[3] = e*.
// Unknowns: the other states and both inputs (n equations, n unknowns). Gives the speed the model holds the drift at.
function steady(M,b,e=0){const n=M.Ad.length,free=[...Array(n).keys()].filter(i=>i!==0&&i!==3);const A=[],y=[];
  for(let i=0;i<n;i++){const row=[];for(const j of free)row.push((i===j?1:0)-M.Ad[i][j]);row.push(-M.Bd[i][0],-M.Bd[i][1]);A.push(row);y.push([M.cd[i]+M.Ad[i][0]*b+M.Ad[i][3]*e-(i===0?b:0)-(i===3?e:0)]);}
  const sol=lsq(A,y,1e-12).map(r=>r[0]);const x=Array(n).fill(0);x[0]=b;x[3]=e;free.forEach((j,k)=>x[j]=sol[k]);return {x,u:[sol[n-2],sol[n-1]]};}
module.exports={steady,fitM,lqiAug,chain,loadEngine,f,setup,obs,aimK,thF,lsq,fit,dlqr,lqi,mm,inv,SC,SU};
