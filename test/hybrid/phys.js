// The hybrid's physical core, first version (decision 11 Oct: a hybrid, integrating the
// existing drift models; this file is the go/no-go test's model, not yet the game's).
//
// What it is: the engine's car reduced to its steady state. Every number is read from the loaded engine (CAR, cfg, the
// surface, VERT, ROAD, the wheel geometry) – no second set of parameters to maintain. Kept from the engine, formula by
// formula (step(), the tyre loop): the tyre law per wheel (normalised slip, mf(), load-dependent peak slip and
// stiffness, load sensitivity of µ, the fall of µ with sliding speed, camber as slip offset and its grip factors, the
// direction blend past the peak), the gravel plough (static wedge and dynamic share), Ackermann, toe and bump steer,
// steering compliance from the aligning torque, rolling resistance, aero drag and downforce, the engine map with
// antilag boost and the drag of a closed throttle, the axle diffs (preload + locking value × axle torque) and the rigid
// centre coupling, the hands' compliance in a slide (gHand), the band's grip and loose share per wheel (lineRel,
// gripFactor; LINE = null as in the measurement's world).
// Reduced: (1) loads – the steady solution of the suspension's spring/anti-roll-bar/tyre-spring equations (vertical())
// with the roll-centre and anti-squat shares, instead of its dynamics; (2) no tyre relaxation, no wheel inertia, no
// steering dynamics (steady state); (3) ruts and road roughness left out (the identification's road is smooth, VERT.amp
// 0; the ruts' slope under the wheels averages out over a lap of the ring); (4) the clutch closed, the gear given.
// Rejected for this test: a single-track model with lumped axles – the load transfer across an axle and the axle diff
// are what set a slide's balance (the inside wheel of a driven axle spins), a lumped axle has neither; it can follow
// once this one says what matters.
'use strict';
function makeModel(E,opt){opt=opt||{};const ruts=opt.ruts!==false;   // opt.ruts false: without the ruts (the engine's NORUT test)
  const CAR=E.CAR,cfg=E.cfg,W=E.WHEELS,VERT=E.VERT,sur=E.SURFX(),RS=E.ROAD[cfg.surface]||E.ROAD.off,g=9.81;
  if(E.rollBars)E.rollBars();
  const L=CAR.a+CAR.b,clamp=(v,a,b)=>v<a?a:v>b?b:v;
  /* steady loads: unknowns z, th, ph, zu[4]; spring + bar force on the body Fs_i = −k_i·ds_i − kb·(ds_i − ds_j),
     ds_i = z + th·rx − ph·ry − zu_i; tyre spring −kt·zu_i = Fs_i; body: ΣFs = 0, ΣFs·rx + m·ax·h·(1−anti) = 0,
     −ΣFs·ry − m·ay·(h − hRc) = 0 (vertical(), dG = 0). Load change = tyre spring force + geometric shares. */
  function loads(ax,ay,v,hz){hz=hz||[0,0,0,0];
    const kA=[VERT.kF,VERT.kF,VERT.kR,VERT.kR],kb=[VERT.karb[0],VERT.karb[0],VERT.karb[1],VERT.karb[1]],kt=VERT.kt;
    const antiL=ax>0?VERT.antiSquat:VERT.antiDive,hRc=VERT.rcF*CAR.b/L+VERT.rcR*CAR.a/L;
    const A=[],y=[];const rx=W.map(w=>w.rx),ry=W.map(w=>w.ry);
    // Fs_i as a row over [z, th, ph, zu0..3]
    const FsRow=i=>{const j=i^1,r=new Array(7).fill(0);const add=(k,c)=>{r[0]+=c;r[1]+=c*rx[k];r[2]-=c*ry[k];r[3+k]-=c;};
      add(i,-kA[i]-kb[i]);add(j,kb[i]);return r;};
    for(let i=0;i<4;i++){const r=FsRow(i).map(v=>-v);r[3+i]-=kt;A.push(r);y.push(-kt*hz[i]);}   // −kt·(zu_i − hz_i) − Fs_i = 0
    {const r=new Array(7).fill(0);for(let i=0;i<4;i++)FsRow(i).forEach((c,k)=>r[k]+=c);A.push(r);y.push(0);}
    {const r=new Array(7).fill(0);for(let i=0;i<4;i++)FsRow(i).forEach((c,k)=>r[k]+=c*rx[i]);A.push(r);y.push(-CAR.m*ax*CAR.h*(1-antiL));}
    {const r=new Array(7).fill(0);for(let i=0;i<4;i++)FsRow(i).forEach((c,k)=>r[k]-=c*ry[i]);A.push(r);y.push(CAR.m*ay*(CAR.h-hRc));}
    const s=solve(A,y),zu=s.slice(3);
    const geoF=CAR.m*ay*(CAR.b/L)*VERT.rcF/CAR.tF,geoR=CAR.m*ay*(CAR.a/L)*VERT.rcR/CAR.tR,geoLong=CAR.m*ax*CAR.h*antiL/L;
    const q=0.5*1.2*v*v,statF=(CAR.m*g*CAR.b/L+q*CAR.ClA*CAR.aeroF)/2,statR=(CAR.m*g*CAR.a/L+q*CAR.ClA*(1-CAR.aeroF))/2;
    const dF=zu.map((z,i)=>-kt*(z-hz[i]));dF[0]+=-geoF-geoLong/2;dF[1]+=geoF-geoLong/2;dF[2]+=-geoR+geoLong/2;dF[3]+=geoR+geoLong/2;
    const Fz=[statF+dF[0],statF+dF[1],statR+dF[2],statR+dF[3]];
    const ds=[0,1,2,3].map(i=>s[0]+s[1]*rx[i]-s[2]*ry[i]-zu[i]);
    const lift=Fz.some(f=>f<0)||ds.some(d=>d<-VERT.travelB||d>VERT.travelD);
    return {Fz:Fz.map(f=>Math.max(0,f)),ph:s[2],comp:ds.map(d=>-d),lift};}
  /* the band's grip and loose share under each wheel on the measuring ring (LINE = null: the band follows the ring's
     centre): wheel i's distance from the ring's centre line from the car's place (path radius rho, slip beta) */
  function wheelRoad(rho,R,beta,turn){const out=[];for(let i=0;i<4;i++){const pr=-beta;   // body x relative to the tangent
      const tc=W[i].rx*Math.cos(pr)-W[i].ry*Math.sin(pr),nc=(W[i].rx*Math.sin(pr)+W[i].ry*Math.cos(pr))*turn;   // nc: towards the centre
      const n=Math.hypot(rho-nc,tc)-R,d=Math.abs(n),loose=RS.band>0?clamp((d-RS.band)/0.8,0,1):0;
      /* the ruts (rutZ, rutSlope; LINE = null: at the ring's centre ± RUT_HALF): their height under the wheel enters the
         loads, their slope a force along the road's normal – outward here: (ux, uy) in the body frame */
      const ut=tc,un=(nc-rho)*turn,um=Math.hypot(ut,un)||1,ux=(ut*Math.cos(beta)-un*Math.sin(beta))/um,uy=(ut*Math.sin(beta)+un*Math.cos(beta))/um;
      const a=(n-E.RUT_HALF)/E.RUT_W,b=(n+E.RUT_HALF)/E.RUT_W,rut=ruts?RS.rut:0;
      out.push({g:RS.band>0?RS.in+(RS.out-RS.in)*loose:1,loose,n,ux,uy,hz:-rut*(Math.exp(-a*a)+Math.exp(-b*b)),sl:rut*(2*a*Math.exp(-a*a)+2*b*Math.exp(-b*b))/E.RUT_W});}return out;}
  /* the tyre loop of step() at steady state for body velocities (vx, vy, r), road-wheel angle delta (S.delta, before
     compliance), wheel speeds w; loads from the steady accelerations (ax, ay); returns forces and what the steady
     state needs */
  function tyres(vx,vy,r,delta,w,ax,ay,road){
    const v=Math.hypot(vx,vy),Ld=loads(ax,ay,v,road.map(q=>q.hz||0)),Fz=Ld.Fz;let dComp=0,out;
    for(let it=0;it<4;it++){   // compliance: dComp = mzF/kSteer (steady); mzF depends on it – fixed point
      let Fxb=0,Fyb=0,Mz=0,mzF=0;const Fxw=[],per=[];
      for(let i=0;i<4;i++){const Wh=W[i];let d=0;
        if(Wh.front){const dc=delta+dComp;if(Math.abs(dc)>1e-4){const Rk=L/Math.tan(dc),dIn=Math.atan(L/(Rk-Math.sign(dc)*Wh.ry));d=dc+CAR.ackermann*(dIn-dc);}}
        const comp=Ld.comp[i],toe=Wh.front?CAR.toeF+CAR.bsF*comp:CAR.toeR+CAR.bsR*comp;d+=Wh.ry>0?-toe:toe;
        const cx=vx-r*Wh.ry,cy=vy+r*Wh.rx,vl=cx*Math.cos(d)+cy*Math.sin(d),vt=-cx*Math.sin(d)+cy*Math.cos(d),vref=Math.max(Math.abs(vl),1.6);
        const kappa=(w[i]*CAR.Rw-vl)/vref,alpha=Math.atan2(vt,vref);
        const lf=Math.max(Fz[i],200)/3200,aPe=sur.aP*Math.pow(lf,0.35),kPe=sur.kP*Math.pow(lf,0.20);
        const camS=(cfg.camber||0)*Math.PI/180,gAbs=(Wh.ry>0?-camS:camS)-Ld.ph*(1-CAR.camberGain),alphaE=alpha+0.25*gAbs;
        const camNeg=(Wh.ry>0?gAbs:-gAbs)*180/Math.PI,fCy=1-0.004*((camNeg-3.5)*(camNeg-3.5)-3.5*3.5),fCx=1-0.012*Math.abs(camNeg);
        const sx=kappa/kPe,sy=-Math.tan(alphaE)/Math.tan(aPe),s=Math.hypot(sx,sy)||1e-6;
        const muScale=Wh.front?1+cfg.bal/100:1-cfg.bal/100,vSl=Math.hypot(vl-w[i]*CAR.Rw,vt),muV=1-0.12*clamp(vSl/12,0,1);
        const mu=sur.mu*road[i].g*muScale*clamp(1.21-0.21*Fz[i]/3200,0.58,1.40)*muV;
        const Fmag=mu*Fz[i]*E.mf(s,sur);let dx=sx/s,dy=sy/s;
        if(s>1){const gx=w[i]*CAR.Rw-vl,gy=-vt,gm=Math.hypot(gx,gy);if(gm>0.05){const wS=clamp((s-1)/1.5,0,1);dx=(1-wS)*dx+wS*gx/gm;dy=(1-wS)*dy+wS*gy/gm;const dm=Math.hypot(dx,dy)||1;dx/=dm;dy/=dm;}}
        let Fx=dx*Fmag*fCx,Fy=dy*Fmag*fCy;
        if(sur.plow>0){const loose=1+RS.loosePlow*road[i].loose,dyn=(sur.plowDyn||0)*loose*vt*vt/(1+(vt/8)*(vt/8));
          const Fpl=Math.min(0.6*Fz[i],sur.plow*loose*Fz[i]*(1-Math.exp(-Math.abs(vt)/1.5))+dyn);Fy-=Math.sign(vt)*Fpl;
          if(kappa<-0.25&&Math.abs(vl)>0.5){const wl=clamp((-kappa-0.25)/0.5,0,1);const Fw=Math.min(0.6*Fz[i]-Fpl,5*sur.plow*loose*Fz[i]*wl);if(Fw>0)Fx-=Math.sign(vl)*Fw;}}
        if(sur.soft>0){const vsx=vl-w[i]*CAR.Rw,vsy=(sur.softLat||0)*vt,vs=Math.hypot(vsx,vsy);if(vs>1e-3){const fs=sur.soft*Fz[i]*(1-Math.exp(-vs/3));Fx-=vsx/vs*fs;Fy-=vsy/vs*fs;}}
        let fx=Fx*Math.cos(d)-Fy*Math.sin(d),fy=Fx*Math.sin(d)+Fy*Math.cos(d);
        if(road[i].sl){const Fn=-Fz[i]*road[i].sl*clamp(1-Math.abs(vt)/2.5,0.3,1);fx+=Fn*road[i].ux;fy+=Fn*road[i].uy;}   // the rut wall (step(): crumbles under a sliding wheel)
        Fxb+=fx;Fyb+=fy;Mz+=Wh.rx*fy-Wh.ry*fx;Fxw.push(Fx);per.push({kappa,alpha,s,Fz:Fz[i],Fx,Fy,vSl});
        if(Wh.front){const tp=0.042*Math.max(0,1-0.85*Math.min(s,1.2));mzF+=-Fy*(0.021+tp);}}
      Fxb-=0.5*1.2*CAR.CdA*vx*Math.abs(vx);Fyb-=0.5*1.2*CAR.CdA*0.6*vy*Math.abs(vy);
      out={Fxb,Fyb,Mz,mzF,Fxw,per,Fz,lift:Ld.lift,dComp};const nc=mzF/CAR.kSteer;if(Math.abs(nc-dComp)<1e-7)break;dComp=nc;}
    return out;}
  /* the engine at steady state: clutch closed in gear gi, boost = throttle × rf (antilag), the drag of a closed throttle */
  function engine(thr,wDrv,gi){const ratio=CAR.gears[gi].r*CAR.final,rpm=wDrv*ratio*60/(2*Math.PI);
    const na=cfg.antilag?CAR.alNa:CAR.naTorque,r1=cfg.antilag?CAR.alRpm1:CAR.boostRpm1,rf=clamp((rpm-CAR.boostRpm0)/(r1-CAR.boostRpm0),0,1);
    const Tmax=E.engineTorque(Math.max(rpm,CAR.idle*0.8)),drag=(0.008*rpm+1.6e-6*rpm*rpm)*clamp((0.20-thr)/0.20,0,1);
    const Te=Tmax*(na*thr+(1-na)*Math.min(thr*rf,thr))-drag;return {Tout:Te*ratio*CAR.eff,rpm};}
  const drive=cfg.drive;let EPSW=0.2;   // EPSW: the plate pack's slip (rad/s) over which its torque turns – Coulomb, regularised; equilibrium() starts wide and narrows it (continuation: with the steep step at once Newton stalled at 15 of 17 AWD points)
  /* residuals of the steady slide on a circle of radius rho at slip beta (left turn: beta < 0, r > 0); unknowns
     z = [v, delta, thr, w0..w3, (AWD) centre torque]; returns the residual vector and the details */
  function resid(z,rho,beta,gi,road){const [v,delta,thr]=z,w=z.slice(3,7),Tc=drive==='awd'?z[7]:0;
    const vx=v*Math.cos(beta),vy=v*Math.sin(beta),r=v/rho,ax=-vy*r,ay=vx*r;
    const T=tyres(vx,vy,r,delta,w,ax,ay,road);
    const drv=drive==='awd'?[0,1,2,3]:drive==='fwd'?[0,1]:[2,3],wD=drv.reduce((a,i)=>a+w[i],0)/drv.length,en=engine(thr,wD,gi);
    const axleT=drive==='awd'?[en.Tout*cfg.split,en.Tout*(1-cfg.split)]:drive==='fwd'?[en.Tout,0]:[0,en.Tout];
    const Td=[axleT[0]/2,axleT[0]/2,axleT[1]/2,axleT[1]/2];
    if(drive==='awd'){Td[0]+=Tc/2;Td[1]+=Tc/2;Td[2]-=Tc/2;Td[3]-=Tc/2;}
    for(let ax2=0;ax2<2;ax2++){const i=ax2*2,j=i+1;if(!drv.includes(i))continue;const lock=axleT[ax2]>=0?cfg.lsd:cfg.lsdCoast,cap=(CAR.lsdPreload+lock*Math.abs(axleT[ax2]))/2;
      const t=cap*Math.tanh((w[j]-w[i])/EPSW);Td[i]+=t;Td[j]-=t;}
    const res=[(T.Fxb/CAR.m-ax),(T.Fyb/CAR.m-ay),T.Mz/CAR.Izz];
    const sc=CAR.m*g/4*CAR.Rw;for(let i=0;i<4;i++)res.push((Td[i]-T.Fxw[i]*CAR.Rw-sur.roll*T.Fz[i]*CAR.Rw*Math.sign(w[i]))/sc);
    if(drive==='awd')res.push(((w[0]+w[1])-(w[2]+w[3]))/20);
    const gH=1-(1-CAR.handSlide)*clamp((Math.abs(beta)-CAR.handSlideB0)/(CAR.handSlideB1-CAR.handSlideB0),0,1);
    const p=delta-T.mzF*(1-CAR.assist)/(gH*CAR.handK*CAR.ratio*CAR.ratio);
    return {res,T,en,p,Td,r};}
  /* Newton with a numerical Jacobian and a backtracking line search */
  /* guess: [v, delta, thr] (the wheels are started here: undriven at their contact patch's speed, driven κ 0.3, then
     0.1 and 0.6 if Newton stalls – with all wheels at +10 % it stalled at 6 of 25 RWD holds) or a full start vector */
  /* near: several steady slides can exist at one radius and slip (AWD at 15°: one with 11° of steering into the corner,
     one with 3° of countersteer and 5 % faster) – with near all three starts are solved and the converged slide
     closest to the guess's speed and steering is returned */
  function equilibrium(rho,R,beta,gi,guess,near){if(guess.length>3)return eq1(rho,R,beta,gi,guess);
    const [v0]=guess,vx=v0*Math.cos(beta),vy=v0*Math.sin(beta),r=v0/rho,drv=drive==='awd'?[0,1,2,3]:drive==='fwd'?[0,1]:[2,3];let best=null;
    const dist=q=>Math.abs(q.v-guess[0])/guess[0]*10+Math.abs(q.delta-guess[1])*57.3/2;
    for(const k of [0.3,0.1,0.6]){const w=W.map((Wh,i)=>(drv.includes(i)?1+k:1)*Math.hypot(vx-r*Wh.ry,vy+r*Wh.rx)/CAR.Rw),g=[...guess,...w];if(drive==='awd')g.push(0);
      const q=eq1(rho,R,beta,gi,g);const qc=q.fn<1e-6,bc=best&&best.fn<1e-6;
      if(!best||(qc&&!bc)||(qc&&bc&&near&&dist(q)<dist(best))||(!qc&&!bc&&q.fn<best.fn))best=q;if(qc&&!near)break;}
    return best;}
  function eq1(rho,R,beta,gi,guess){const road=wheelRoad(rho,R,beta,1);
    let z=guess.slice();const n=z.length,nrm=a=>Math.sqrt(a.reduce((s,x)=>s+x*x,0));let f,fn,it=0;
    for(const eps of [20,5,1,0.2]){EPSW=eps;f=resid(z,rho,beta,gi,road).res;fn=nrm(f);
    for(let k=0;k<80&&fn>1e-9;k++,it++){const J=[];for(let k=0;k<n;k++){const h=k===0?1e-4*Math.max(1,z[0]):k<3?1e-6:k<7?1e-4:1e-2;const z2=z.slice();z2[k]+=h;const f2=resid(z2,rho,beta,gi,road).res;J.push(f2.map((x,i)=>(x-f[i])/h));}
      const A=f.map((_,i)=>J.map(col=>col[i])),dz=solve(A,f.map(x=>-x));if(!dz.every(isFinite))break;
      {const c=Math.max(1,Math.abs(dz[0])/(0.03*z[0]),Math.abs(dz[1])/0.035,Math.abs(dz[2])/0.1);if(c>1)for(let k=0;k<n;k++)dz[k]/=c;}   // a local step (≤ 3 % speed, 2° steering, 0.1 throttle): near a fold of the slide's family the full step jumped to another slide
      let lam=1,ok=false;for(let ls=0;ls<12;ls++){const z2=z.map((x,k)=>x+lam*dz[k]);z2[2]=clamp(z2[2],-0.5,1.5);if(z2[0]>1){const f2=resid(z2,rho,beta,gi,road).res,n2=nrm(f2);if(n2<fn){z=z2;f=f2;fn=n2;ok=true;break;}}lam/=2;}
      if(!ok)break;}}
    const d=resid(z,rho,beta,gi,road);return {z,fn,it,v:z[0],delta:z[1],thr:z[2],w:z.slice(3,7),p:d.p,T:d.T,rpm:d.en.rpm,road,Td:d.Td,gi,rho,beta,Tc:drive==='awd'?z[7]:0};}
  /* THE DYNAMICS (for the linearisation; decision 11 Oct): x = [vx, vy, r, w0..w3, boost], u = [delta, thr]. Kept: the
     wheels' inertia with the engine's reflected through the gear (clutch closed), the turbo's boost lag (antilag), the
     axle diffs as regularised Coulomb plates (EPSW 0.2 rad/s), the rigid centre as a stiff coupling around the steady
     slide's torque (Kc 3000 N·m per rad/s, about 4 ms – the engine's constraint is exact). Reduced: the loads follow the
     accelerations at once (the engine's body rolls and pitches on springs and dampers, 1.7–2.1 Hz – for 0.5 s ahead the
     lag is the first thing to add if the responses disagree), no tyre relaxation (0.45 m: 20–30 ms at these speeds),
     the steering compliance at once (20 ms in the engine). The road under the wheels stays the steady slide's. */
  /* with q.steer: x gains [delta, dDelta] and u = [p, thr] – the steering as step() moves it: the hands at the target p
     through the arms' compliance (relaxed in a slide, gHand; their rate limit left out – linear), the column's inertia
     and damping, the tyres' aligning torque */
  function deriv(x,u,q){const [vx,vy,r]=x,w=x.slice(3,7),boost=x[7],delta=q.steer?x[8]:u[0],thr=u[1],gi=q.gi,ratio=CAR.gears[gi].r*CAR.final;
    let acc=q.acc0||[-vy*r,vx*r],T;for(let k=0;k<4;k++){T=tyres(vx,vy,r,delta,w,acc[0],acc[1],q.road);acc=[T.Fxb/CAR.m,T.Fyb/CAR.m];}
    const drv=drive==='awd'?[0,1,2,3]:drive==='fwd'?[0,1]:[2,3],wD=drv.reduce((a,i)=>a+w[i],0)/drv.length,rpm=wD*ratio*60/(2*Math.PI);
    const na=cfg.antilag?CAR.alNa:CAR.naTorque,r1=cfg.antilag?CAR.alRpm1:CAR.boostRpm1,rf=clamp((rpm-CAR.boostRpm0)/(r1-CAR.boostRpm0),0,1);
    const tau=cfg.antilag?(0.10+0.12*(1-rf)):(0.28+0.55*(1-rf)),Tmax=E.engineTorque(Math.max(rpm,CAR.idle*0.8)),drag=(0.008*rpm+1.6e-6*rpm*rpm)*clamp((0.20-thr)/0.20,0,1);
    const Te=Tmax*(na*thr+(1-na)*Math.min(boost,thr))-drag,Tout=Te*ratio*CAR.eff;
    const axleT=drive==='awd'?[Tout*cfg.split,Tout*(1-cfg.split)]:drive==='fwd'?[Tout,0]:[0,Tout],Td=[axleT[0]/2,axleT[0]/2,axleT[1]/2,axleT[1]/2];
    if(drive==='awd'){const Tc=q.Tc0+3000*((w[2]+w[3])/2-(w[0]+w[1])/2);Td[0]+=Tc/2;Td[1]+=Tc/2;Td[2]-=Tc/2;Td[3]-=Tc/2;}
    for(let a2=0;a2<2;a2++){const i=a2*2,j=i+1;if(!drv.includes(i))continue;const lock=axleT[a2]>=0?cfg.lsd:cfg.lsdCoast,cap=(CAR.lsdPreload+lock*Math.abs(axleT[a2]))/2,t=cap*Math.tanh((w[j]-w[i])/0.2);Td[i]+=t;Td[j]-=t;}
    const Iref=CAR.Ieng*ratio*ratio/drv.length,dx=[acc[0]+vy*r,acc[1]-vx*r,T.Mz/CAR.Izz];
    for(let i=0;i<4;i++){const I=CAR.Iw+(drv.includes(i)?Iref:0);dx.push((Td[i]-T.Fxw[i]*CAR.Rw-sur.roll*T.Fz[i]*CAR.Rw*Math.sign(w[i]))/I);}
    dx.push((thr*rf-boost)/tau);
    if(q.steer){const dd=x[9],Rr=CAR.ratio,bA=Math.abs(Math.atan2(vy,Math.abs(vx))),gH=1-(1-CAR.handSlide)*clamp((bA-CAR.handSlideB0)/(CAR.handSlideB1-CAR.handSlideB0),0,1);
      const Th=clamp(gH*(CAR.handK*(u[0]*Rr-delta*Rr)-CAR.handD*dd*Rr),-CAR.handTmax,CAR.handTmax);dx.push(dd,(Th*Rr/(1-CAR.assist)+T.mzF-CAR.steerC*dd)/CAR.steerJ);}
    return dx;}
  /* the linearisation at a steady slide q (equilibrium()'s result): central differences; returns A (8×8), B (8×2) and
     C (3×8: β, r, v) */
  function linearise(q,steer){const vx=q.v*Math.cos(q.beta),vy=q.v*Math.sin(q.beta),r=q.v/q.rho;const rpm=q.rpm,r1=cfg.antilag?CAR.alRpm1:CAR.boostRpm1,rf=clamp((rpm-CAR.boostRpm0)/(r1-CAR.boostRpm0),0,1);
    const x0=[vx,vy,r,...q.w,q.thr*rf],u0=[steer?q.p:q.delta,q.thr],ctx={gi:q.gi,road:q.road,Tc0:q.Tc,acc0:[-vy*r,vx*r],steer};if(steer)x0.push(q.delta,0);
    const f0=deriv(x0,u0,ctx),hx=[1e-4,1e-4,1e-5,1e-3,1e-3,1e-3,1e-3,1e-5,1e-6,1e-5],hu=[1e-5,1e-5],nx=x0.length;
    const col=(xp,up,xm,um,h)=>{const a=deriv(xp,up,ctx),b=deriv(xm,um,ctx);return a.map((v,i)=>(v-b[i])/(2*h));};
    const A=x0.map(()=>[]),B=x0.map(()=>[]);
    for(let j=0;j<nx;j++){const xp=x0.slice(),xm=x0.slice();xp[j]+=hx[j];xm[j]-=hx[j];const c=col(xp,u0,xm,u0,hx[j]);c.forEach((v,i)=>A[i][j]=v);}
    for(let j=0;j<2;j++){const up=u0.slice(),um=u0.slice();up[j]+=hu[j];um[j]-=hu[j];const c=col(x0,up,x0,um,hu[j]);c.forEach((v,i)=>B[i][j]=v);}
    const v2=q.v*q.v,C=[[-vy/v2,vx/v2,0,0,0,0,0,0],[0,0,1,0,0,0,0,0],[vx/q.v,vy/q.v,0,0,0,0,0,0]];if(steer){C.forEach(c=>c.push(0,0));C.push([0,0,0,0,0,0,0,0,1,0]);}
    return {A,B,C,f0,x0,u0};}
  return {loads,tyres,engine,resid,equilibrium,wheelRoad,deriv,linearise};}
function solve(A,y){const n=A.length,M=A.map((r,i)=>[...r,y[i]]);
  for(let c=0;c<n;c++){let p=c;for(let r=c+1;r<n;r++)if(Math.abs(M[r][c])>Math.abs(M[p][c]))p=r;[M[c],M[p]]=[M[p],M[c]];const d=M[c][c];if(Math.abs(d)<1e-300)return new Array(n).fill(NaN);
    for(let r=0;r<n;r++){if(r===c)continue;const f=M[r][c]/d;if(f!==0)for(let k=c;k<=n;k++)M[r][k]-=f*M[c][k];}}
  return M.map((r,i)=>r[n]/r[i]);}
module.exports={makeModel,solve};
