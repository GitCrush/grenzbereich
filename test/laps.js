// Race autopilot over several laps – does the learning master the limit? node laps.js track surface drive pace [laps=6]
// Per lap: time, off track, catch active, learned scale (minimum and mean).
const {loadEngine}=require('./load');const f=(x,d=1)=>(+x).toFixed(d);const [tr,surface,drive,pace,nL]=process.argv.slice(2);const NL=+(nL||6);
const E=loadEngine();Object.assign(E.cfg,{surface,drive,abs:false,auto:true,steer:'direct',uniform:false,profile:true,driven:true,hazards:'off'});E.VERT.amp=1;E.applyDriveSetup(drive);E.selectTrack(tr);
E.AP.on=true;E.AP.pace=+pace;E.AP.drift=0;E.apEnsure();while(E.AP.job)E.apRun(1e9);E.apEnsure();while(E.AP.lineJob)E.apLineRun(1e9);E.apEnsure();if(!process.env.NOPRE){while(E.AP.preJob)E.apPreRun(1e9);E.apEnsure();}
const S=E.S,T=E.TRACK,L=T.pts[T.N-1].s;let t=0,laps=0,prevS=T.pts[S.idx].s,tL=0,off=0,cat=0;const out=[];
while(t<120*NL&&laps<NL+1){if(Math.round(t/E.DT)%90===0)E.apEnsure();const c=E.ghostCommand(E.IN.brk,E.DT,'catch');E.IN.steer=c.steer;E.IN.thr=c.thr;E.IN.brk=c.brk;E.IN.hb=c.hb||0;E.step(E.DT);t+=E.DT;tL+=E.DT;
  if(!S.onTrack)off+=E.DT;if((E.AP.wC||0)>0)cat+=E.DT;const s=T.pts[S.idx].s;
  if(s<prevS-L/2){if(laps>0){const sc=E.AP.lrn?E.AP.lrn.scale:null;let mn=1,mean=0;if(sc){for(const x of sc){mn=Math.min(mn,x);mean+=x;}mean/=sc.length;}
      out.push('lap '+laps+': '+f(tL)+' s, off '+f(off)+' s, catch '+f(cat)+' s, scale min '+f(mn,3)+' mean '+f(mean,3));}laps++;tL=0;off=0;cat=0;}prevS=s;}
const H=E.AP.pre?E.AP.pre.hist:null;console.log(tr,surface,drive,'pace',pace,'| trial laps',H?JSON.stringify(H):'none','| '+out.join(' | '));
