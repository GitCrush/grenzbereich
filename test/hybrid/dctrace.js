// A trace of the drift controller through a corner (decision 11 Oct: the computed table held its ring, the corners swung
// ±9° around 8° under the target): every 0.1 s inside the controller's corners – slip, its reference, the wanted angle,
// road-wheel angle, hands' target, throttle, speed, offset from the line, wheel slip of the driven wheels, gear.
// node hybrid/dctrace.js   env: OV="55,200" (R,straight; 0 = circuit), TRACK (a named track), SEED, LAP (default 2; 0 the first),
//   SR="s0,s1" (print every 0.1 s on that stretch, controller on or not), DRIVE, SETA, LSFILE
const {loadEngine}=require('../load');const f=(x,d=1)=>(x>=0?' ':'')+(+x).toFixed(d);const E=loadEngine();
if(process.env.DRIVE){E.cfg.drive=process.env.DRIVE;E.applyDriveSetup(E.cfg.drive);}E.AP.drift=1;Object.assign(E.AP,{dcUse:true},JSON.parse(process.env.SETA||'{}'));
const [oR,oS]=(process.env.OV||'55,200').split(',').map(Number);if(process.env.SEED){E.TRACKS.random=E.randomTrackDef(+process.env.SEED);E.selectTrack('random');}else if(process.env.TRACK)E.selectTrack(process.env.TRACK);else if(oR){E.TRACKS.oval={name:'Oval',type:'seg',segs:oS>0?[{s:oS},{R:oR,a:180},{s:oS},{R:oR,a:180}]:[{R:oR,a:360}],w:10.5,ex:null};E.selectTrack('oval');}else E.selectTrack('rundkurs');
E.AP.on=true;E.apPrepare();const S=E.S,T=E.TRACK,N=T.N,Ls=T.pts[N-1].s,DT=E.DT,C=E.AP.dcC,lapW=+(process.env.LAP||2);
let t=0,laps=0,prev=T.pts[S.idx].s,n=0;
while(laps<=lapW&&t<400){if(n%180===0)E.apEnsure();const c=E.ghostCommand(E.AP.pedBrk||0,DT,'catch');E.AP.pedBrk=c.brk;E.IN.steer=c.steer;E.IN.thr=c.thr;E.IN.brk=c.brk;E.IN.hb=c.hb||0;E.step(DT);t+=DT;n++;
  const s=T.pts[S.idx].s;if(s<prev-Ls/2)laps++;prev=s;if(laps!==lapW||n%36)continue;const st=E.AP.dcS,on=!!(st&&st.on),sR=process.env.SR&&process.env.SR.split(',').map(Number);if(sR?s<sR[0]||s>sR[1]:!on&&!(C&&C.dc[S.idx]))continue;
  const P=T.pts[S.idx],lat=(S.x-P.x)*P.nx+(S.y-P.y)*P.ny,b=Math.atan2(S.vy,Math.abs(S.vx)),v=Math.hypot(S.vx,S.vy),vl=S.vx;
  const P2=T.pts[S.idx];console.log(`s${f(P2.s,0)} R${f(P2.R||0,0)} t${f(t,1)} ${on?'DC':'--'} β${f(b*57.3)} ref${f(st&&st.bR!==undefined?st.bR*57.3:0)} want${f((E.AP.brNow||0)*57.3)} δ${f(S.delta*57.3)} p${f((S.dLP||0)*57.3)} thr${f(E.IN.thr,2)} v${f(v*3.6,0)} plan${f((E.AP.prof?E.AP.prof.v[S.idx]:0)*3.6,0)} lat${f(lat)} κR${f((S.w[2]*E.CAR.Rw-vl)/Math.max(vl,1),2)} g${S.gi}/${st&&st.gi||0}${E.AP.dcAb?' GIVEN UP':''}`);}
