// Drift and grip scenarios for the autopilot, two at a time, with the bars of the acceptance suite plus steering activity.
//   node drift-suite.js [drift=1] [tag]        env passed through: SETA, SETC, REACT (driver reaction 0..1), SETS (gravel tyre)
// Bars: on the road (off 0.0 s) and at most 3 visible steering corrections per second (reversals of the hands by more
// than 10° at the steering wheel). Slide length and the share of the planned slide stretches driven sideways are reported,
// not barred – long slides on circuits and stages are open work (TODO).
const {spawn}=require('child_process');const drift=process.argv[2]||'1',tag=process.argv[3]||'';
const sc=drift==='0'?[['circuit AWD',{}],['circuit RWD',{DRIVE:'rwd'}],['circuit FWD',{DRIVE:'fwd'}],['stage 19029',{SEED:'19029'}],['stage 9968',{SEED:'9968'}],['stage 4711',{SEED:'4711'}],['stage FWD',{DRIVE:'fwd',TR:'etappe'}],['stage AWD',{TR:'etappe'}]]
  :[['circuit AWD',{}],['circuit RWD',{DRIVE:'rwd'}],['circuit FWD',{DRIVE:'fwd'}],['stage 19029',{SEED:'19029'}],['stage 9968',{SEED:'9968'}],['stage 4711',{SEED:'4711'}],['oval R 83 m',{OVAL:'83,250'}],['stage RWD',{DRIVE:'rwd',TR:'etappe'}]];
let act=0,fails=0;const out={},q=sc.slice();
function next(){if(!q.length){if(!act){for(const [k] of sc)console.log(out[k]);console.log(fails?fails+' FAIL':'all PASS');process.exitCode=fails?1:0;}return;}
  const [k,ex]=q.shift();act++;const env=Object.assign({},process.env,ex,{TAG:(tag+' '+k).trim()});
  const p=spawn('node',[__dirname+'/drift-bench.js',ex.TR||(ex.SEED?'random':'rundkurs'),drift],{cwd:__dirname,env});
  let s='';p.stdout.on('data',d=>s+=d);p.stderr.on('data',d=>s+=d);
  p.on('close',()=>{const line=s.trim().split('\n').slice(-1)[0]||'(no output)';const off=+(line.match(/off ([\d.]+)/)||[0,99])[1],rev=+(line.match(/hand rev\/s ([\d.]+)/)||[0,99])[1];
    const ok=off===0&&rev<=3;if(!ok)fails++;out[k]=(ok?'PASS ':'FAIL ')+line;act--;next();});}
next();next();
