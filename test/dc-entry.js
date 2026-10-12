// The drift controller's corners one by one: how each slide was started, held and ended (decision 10 Oct – a lap of an
// oval is two corners, and the lap figures of drift-bench mixed a good corner with a lost one).
// node dc-entry.js   env: OVALS="40,150 83,250" (R,straight – straight 0: a ring; default four ovals), SEED / TRACK instead (one stage),
//   NL laps after the first (default 2), DRIVE, REACT, SETA (JSON for AP; dcUse is set), TAG, V=1 a line per corner.
// Per corner (a passage through a stretch the plan gives the controller, AP.dcC.segs): engaged or not; time from the
// corner's start to the engagement; time from the engagement to 75 % of the target; the largest slip past the target;
// the share of the held part (target at 90 % of the corner's largest) within ±4°; given up (AP.dcAb); the least room to
// the edge; time off the road. Summary per track: corners, engaged, built (75 % reached), given up, off, mean ±4 share.
const {loadEngine}=require('./load');const f=(x,d=1)=>(+x).toFixed(d);
const nl=+(process.env.NL||2),V=process.env.V==='1';
const list=process.env.SEED?[{seed:+process.env.SEED}]:process.env.TRACK?[{tr:process.env.TRACK}]:(process.env.OVALS||'40,150 55,200 83,250 120,250').split(' ').map(o=>({oval:o}));
let all={n:0,eng:0,built:0,ab:0,off:0,inb:0,nh:0,es:0,es2:0};
for(const L0 of list){const E=loadEngine();if(process.env.DRIVE){E.cfg.drive=process.env.DRIVE;E.applyDriveSetup(E.cfg.drive);}E.AP.drift=+(process.env.DRIFT||1);
  if(process.env.REACT)E.applyReaction(+process.env.REACT);Object.assign(E.AP,{dcUse:true},process.env.SETA?JSON.parse(process.env.SETA):{});
  let name;if(L0.oval){const [oR,oS]=L0.oval.split(',').map(Number);E.TRACKS.oval={name:'Oval',type:'seg',segs:oS>0?[{s:oS},{R:oR,a:180},{s:oS},{R:oR,a:180}]:[{R:oR,a:360}],w:10.5,ex:null};E.selectTrack('oval');name=(oS>0?'oval ':'ring ')+L0.oval;}
  else if(L0.seed){E.TRACKS.random=E.randomTrackDef(L0.seed);E.selectTrack('random');name='seed '+L0.seed;}else{E.selectTrack(L0.tr);name=L0.tr;}
  if(process.env.QI)E.DC_QI=JSON.parse(process.env.QI);if(process.env.AMP!==undefined)E.VERT.amp=+process.env.AMP;if(process.env.NODRIVEN)E.cfg.driven=false;E.AP.on=true;E.apPrepare();const S=E.S,T=E.TRACK,N=T.N,Ls=T.pts[N-1].s,DT=E.DT;const C=E.AP.dcC;if(!C||!C.segs.length){console.log((process.env.TAG||'')+' '+name,'| no controller corners');continue;}
  const segOf=new Int16Array(N).fill(-1);C.segs.forEach(([a,b],k)=>{for(let i=a;;i=(i+1)%N){segOf[i]=k;if(i===b)break;}});
  const bM=C.segs.map(([a,b])=>{let m=0;for(let i=a;;i=(i+1)%N){m=Math.max(m,Math.abs(C.bD[i]));if(i===b)break;}return m;});
  let t=0,laps=0,prev=T.pts[S.idx].s,n=0,cur=null;const res=[];
  const close=()=>{if(cur){res.push(cur);cur=null;}};
  while(t<300*(nl+1)&&laps<nl+1){if(n%180===0)E.apEnsure();const c=E.ghostCommand(E.AP.pedBrk||0,DT,'catch');E.AP.pedBrk=c.brk;E.IN.steer=c.steer;E.IN.thr=c.thr;E.IN.brk=c.brk;E.IN.hb=c.hb||0;E.step(DT);t+=DT;n++;
    const s=T.pts[S.idx].s;if(s<prev-Ls/2){laps++;if(C.segs.length===1&&C.segs[0][0]===0&&C.segs[0][1]===N-1&&E.AP.dcAb&&!(E.AP.dcS&&E.AP.dcS.on))E.AP.dcAb=false;}prev=s;if(laps<1){continue;}   // a ring: the corner never ends, a slide given up on the first lap is retried on the next
    const k=segOf[S.idx];if(k<0||(cur&&cur.k!==k)){if(cur&&(k<0?++cur.out>30:true))close();if(k<0)continue;}else if(cur)cur.out=0;
    if(!cur)cur={k,t:0,tE:-1,tB:-1,over:0,ab:false,edge:99,off:0,inb:0,nh:0,out:0,vE:0,vP:0};
    const P=T.pts[S.idx],lat=(S.x-P.x)*P.nx+(S.y-P.y)*P.ny,edge=P.w-Math.abs(lat),b=Math.atan2(S.vy,Math.abs(S.vx)),br=E.AP.brNow||0,on=!!(E.AP.dcS&&E.AP.dcS.on);
    cur.t+=DT;cur.edge=Math.min(cur.edge,edge);if(!S.onTrack)cur.off+=DT;if(E.AP.dcAb&&!cur.ab){cur.ab=true;cur.abAt=`at ${f(cur.t,1)}s β ${f(b*57.3,0)}° tgt ${f(br*57.3,0)}° lat ${f(lat,1)}`;}
    if(on&&cur.tE<0){cur.tE=cur.t;cur.vE=Math.hypot(S.vx,S.vy)*3.6;cur.vP=(E.AP.prof?E.AP.prof.v[S.idx]:0)*3.6;}
    if(cur.tE>=0&&cur.tB<0&&Math.abs(b)>=0.75*bM[k]&&Math.sign(b)===Math.sign(br||b))cur.tB=cur.t-cur.tE;
    if(Math.sign(b)===Math.sign(C.bD[S.idx]))cur.over=Math.max(cur.over,(Math.abs(b)-bM[k])*57.3);
    if(Math.abs(C.bD[S.idx])>=0.9*bM[k]){cur.nh+=DT;if(Math.abs(b-C.bD[S.idx])<4/57.3)cur.inb+=DT;const eb=(Math.abs(b)-Math.abs(C.bD[S.idx]))*57.3;cur.es=(cur.es||0)+eb*DT;cur.es2=(cur.es2||0)+eb*eb*DT;}}
  close();const st={n:res.length,eng:0,built:0,ab:0,off:0,inb:0,nh:0,es:0,es2:0};
  for(const r of res){st.es+=r.es||0;st.es2+=r.es2||0;if(r.tE>=0)st.eng++;if(r.tB>=0)st.built++;if(r.ab)st.ab++;st.off+=r.off;st.inb+=r.inb;st.nh+=r.nh;
    if(V)console.log('   corner',r.k,'| eng',r.tE<0?'no':f(r.tE,2)+'s at '+f(r.vE,0)+'/'+f(r.vP,0),'| build',r.tB<0?'–':f(r.tB,2)+'s','| over',f(r.over,0)+'°','| ±4',f(100*r.inb/Math.max(1e-9,r.nh),0)+'%','| err',r.nh>0?f(r.es/r.nh,1)+'±'+f(Math.sqrt(Math.max(0,r.es2/r.nh-(r.es/r.nh)**2)),1)+'°':'–','|',r.ab?'GIVEN UP '+r.abAt:'held','| edge',f(r.edge,1),'| off',f(r.off,1));}
  for(const kk in all)all[kk]+=st[kk];
  console.log((process.env.TAG||'')+' '+name.padEnd(14),E.cfg.drive,'| corners',st.n,'engaged',st.eng,'built',st.built,'given up',st.ab,'| ±4',f(100*st.inb/Math.max(1e-9,st.nh),0)+'%','| err',f(st.es/Math.max(1e-9,st.nh),1)+'±'+f(Math.sqrt(Math.max(0,st.es2/Math.max(1e-9,st.nh)-(st.es/Math.max(1e-9,st.nh))**2)),1)+'°','| off',f(st.off,1));}
if(list.length>1)console.log((process.env.TAG||'')+' ALL'.padEnd(15),'| corners',all.n,'engaged',all.eng,'built',all.built,'given up',all.ab,'| ±4',f(100*all.inb/Math.max(1e-9,all.nh),0)+'%','| off',f(all.off,1));
