// Bit-identity of two engine builds (decision 11 Oct: refactors of step() – the tyre law factored out for the slide
// model – must not change a single bit). Runs the same scripted drives on both and compares the whole state (S, VERT)
// at every second; prints the first difference.
// node same.js [REF=engine_10b.js] [NEW=engine.js]   (ENGINE files in test/)
const vm=require('vm');const ref=process.argv[2]||'engine_10b.js',neu=process.argv[3]||'engine.js';
const {loadEngine}=require('./load');
const load=f=>{process.env.ENGINE=f;return loadEngine();};
const clamp=(v,a,b)=>v<a?a:v>b?b:v;
// a drive: throttle, steering, brake and handbrake patterns over 24 s – limit, slides, lock-ups, off the road
const input=(t,k)=>{const ph=t%12;return {thr:ph<4?clamp(t/2,0,1):ph<6?0.3:ph<8?0:0.8,steer:0.6*Math.sin(t*(0.7+0.2*k))+(ph>8&&ph<9?0.8:0),brk:ph>6&&ph<7.5?0.9:0,hb:ph>9.5&&ph<10?1:0};};
const scen=[];
for(const drive of ['awd','rwd','fwd'])for(const surface of ['gravel','tarmac','snow','sand'])scen.push({drive,surface,weather:'clear',track:'rundkurs'});
scen.push({drive:'awd',surface:'tarmac',weather:'rain',track:'rundkurs'},{drive:'rwd',surface:'gravel',weather:'rain',track:'random'},{drive:'awd',surface:'gravel',weather:'clear',track:'random',abs:true,tc:true});
let bad=0;
for(const [k,s] of scen.entries()){const E=[load(ref),load(neu)];
  for(const e of E){Object.assign(e.cfg,{surface:s.surface,drive:s.drive,weather:s.weather,abs:!!s.abs,tc:!!s.tc,auto:true,steer:'direct',hazards:'heavy',uniform:false,profile:true,driven:true});e.applyDriveSetup(s.drive);
    if(s.track==='random'){e.TRACKS.random=e.randomTrackDef(1234+k);e.selectTrack('random');}else e.selectTrack(s.track);e.VERT.amp=1;}
  const DT=E[0].DT,n=Math.round(24/DT);let diff=null;
  for(let i=0;i<n&&!diff;i++){const t=i*DT,u=input(t,k);for(const e of E){Object.assign(e.IN,u);e.step(DT);}
    if(i%360===359||i===n-1){const a=JSON.stringify([E[0].S,E[0].VERT]),b=JSON.stringify([E[1].S,E[1].VERT]);if(a!==b){let j=0;while(a[j]===b[j])j++;diff=`at ${(t).toFixed(2)} s: …${a.slice(Math.max(0,j-60),j+40)}… vs …${b.slice(Math.max(0,j-60),j+40)}…`;}}}
  const v=Math.hypot(E[0].S.vx,E[0].S.vy)*3.6;console.log(`${s.drive} ${s.surface} ${s.weather} ${s.track}${s.abs?' abs+tc':''}: ${diff?'DIFFERENT '+diff:'identical'} (end ${v.toFixed(0)} km/h, on road ${E[0].S.onTrack})`);if(diff)bad++;}
console.log(bad?`${bad} scenario(s) differ`:'ALL IDENTICAL');process.exit(bad?1:0);
