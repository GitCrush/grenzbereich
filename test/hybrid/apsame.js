// The autopilot with the drift controller off – old and new build drive the same bits?
// (decision 11 Oct: the slide model changes nothing unless AP.dcUse is on). 90 s on the circuit, drift 0 and 1, AWD/RWD.
// Since 12 Oct the controller is on by default for RWD (AP.dcUse 'auto'): forced off here for the new build, so the test
// still shows that the older layer drives as in 10b. Likewise the line controller (AP.lcUse, on since 12b).
const {loadEngine}=require('../load');const ld=f=>{process.env.ENGINE=f;return loadEngine();};let bad=0;
for(const [drive,drift] of [['awd',0],['awd',1],['rwd',1]]){const E=[ld('engine_10b.js'),ld('engine.js')];
  if(E[1].AP.dcUse!==undefined)E[1].AP.dcUse=false;if(E[1].AP.lcUse!==undefined)E[1].AP.lcUse=false;for(const e of E){e.cfg.drive=drive;e.applyDriveSetup(drive);e.selectTrack('rundkurs');e.AP.drift=drift;e.AP.on=true;e.apPrepare();}
  let diff=null;for(let i=0;i<90*360&&!diff;i++){for(const e of E){if(i%180===0)e.apEnsure();const c=e.ghostCommand(e.AP.pedBrk||0,e.DT,'catch');e.AP.pedBrk=c.brk;e.IN.steer=c.steer;e.IN.thr=c.thr;e.IN.brk=c.brk;e.IN.hb=c.hb||0;e.step(e.DT);}
    if(i%3600===3599){const a=JSON.stringify(E[0].S),b=JSON.stringify(E[1].S);if(a!==b)diff='at '+((i+1)/360).toFixed(0)+' s';}}
  console.log(drive,'drift',drift,':',diff?'DIFFERENT '+diff:'identical');if(diff)bad++;}
process.exit(bad?1:0);
