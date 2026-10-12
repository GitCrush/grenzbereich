// The drift controller's corners on a track (12 Oct): the candidates (dcCorners – the stretches its slides are planned on),
// the corner check's verdict on each (engaged, given up, off the road, largest slip) and the corners kept; SEG=s (a
// candidate's start, m) prints the check's drive of that corner every 0.1 s (AP.tChkLog): corner index, s, slip, target,
// speed, plan, offset from the centre, road-wheel angle, throttle, brake, gear, controller on, its slip reference, yaw rate.
// node hybrid/corners.js   env: TRACK or SEED, DRIVE, SETA (e.g. '{"dcRR":2.5,"dcMinT":2}' – the looser corner criteria), SEG
const {loadEngine}=require('../load');const E=loadEngine();
if(process.env.DRIVE){E.cfg.drive=process.env.DRIVE;E.applyDriveSetup(E.cfg.drive);}E.AP.drift=1;Object.assign(E.AP,{dcUse:true,tChkLog:[]},JSON.parse(process.env.SETA||'{}'));
if(process.env.SEED){E.TRACKS.random=E.randomTrackDef(+process.env.SEED);E.selectTrack('random');}else E.selectTrack(process.env.TRACK||'rundkurs');
E.AP.on=true;E.apPrepare();const T=E.TRACK,C=E.AP.dcC,seg=+(process.env.SEG||0),sOf=i=>Math.round(T.pts[i].s);
const log=E.AP.tChkLog;E.AP.tChkLog=null;const kept=C.segs.map(([a,b])=>[sOf(a),sOf(b)]),chk=C.chk||[];
E.AP.dcC=null;const cand=E.dcCorners(E.AP.env).segs.map(([a,b])=>[sOf(a),sOf(b)]);
cand.forEach((c,k)=>{const r=chk[k];console.log(`corner s ${c[0]}–${c[1]} m: ${r?(r.ok?'kept':'removed')+` (engaged ${r.eng}, given up ${r.ab}, off ${r.off}, slip ≤ ${(r.bMax*57.3).toFixed(0)}°)`:'not checked'}`);});
console.log('kept',JSON.stringify(kept));
if(seg){console.log('i s β target v plan lat δ thr brk gear dc ref r');for(const r of log)if(Math.abs(T.pts[r[0]].s-seg)<5)console.log(r.join(' '));}
