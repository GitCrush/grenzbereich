// The plan along a stretch (decision 11 Oct, stage 4711 s ≈ 1642 m): per track point between S0 and S1 – the road's
// curvature (signed radius), the racing line's offset and radius, the planned speed, the slide wanted, the half width.
// node hybrid/linehere.js S0 S1   env: SEED/TRACK, DRIVE, DRIFT, SETA, LSFILE
const {loadEngine}=require('../load');const f=(x,d=1)=>(x>=0?' ':'')+(+x).toFixed(d);const E=loadEngine();if(process.env.DRIVE){E.cfg.drive=process.env.DRIVE;E.applyDriveSetup(E.cfg.drive);}
E.AP.drift=+(process.env.DRIFT||1);if(process.env.SETA)Object.assign(E.AP,JSON.parse(process.env.SETA));
if(process.env.SEED){E.TRACKS.random=E.randomTrackDef(+process.env.SEED);E.selectTrack('random');}else E.selectTrack(process.env.TRACK||'rundkurs');
E.AP.on=true;E.apPrepare();const T=E.TRACK,PL=E.AP.line,P=E.AP.prof,s0=+process.argv[2],s1=+process.argv[3];
for(let i=0;i<T.N;i++){const p=T.pts[i];if(p.s<s0||p.s>s1||i%2)continue;const l=PL[i];
  console.log(`s ${f(p.s,0)} road R ${f((p.ks||0)*Math.min(p.R||1e4,9999),0)} | line o ${f(l.o)} R ${f((l.ks||Math.sign(p.ks||0))*Math.min(l.R,9999),0)} | plan ${f(P.v[i]*3.6,0)} | want ${f((P.b?P.b[i]:0)*57.3)} | w ${f(p.w)}`);}
