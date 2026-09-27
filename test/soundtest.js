const {loadEngine}=require('./load');const {OfflineAudioContext}=require('node-web-audio-api');const fs=require('fs');
function wav(path,buf){const n=buf.length,ch=buf.numberOfChannels,sr=buf.sampleRate;const out=Buffer.alloc(44+n*ch*2);
  out.write('RIFF',0);out.writeUInt32LE(36+n*ch*2,4);out.write('WAVE',8);out.write('fmt ',12);out.writeUInt32LE(16,16);out.writeUInt16LE(1,20);out.writeUInt16LE(ch,22);out.writeUInt32LE(sr,24);out.writeUInt32LE(sr*ch*2,28);out.writeUInt16LE(ch*2,32);out.writeUInt16LE(16,34);out.write('data',36);out.writeUInt32LE(n*ch*2,40);
  const chans=[];for(let c=0;c<ch;c++)chans.push(buf.getChannelData(c));let o=44;for(let i=0;i<n;i++)for(let c=0;c<ch;c++){let v=Math.max(-1,Math.min(1,chans[c][i]));out.writeInt16LE(Math.round(v*32767),o);o+=2;}fs.writeFileSync(path,out);}
async function demo(name,surface,script,dur){
  const sr=44100,off=new OfflineAudioContext(2,Math.ceil(sr*dur),sr);
  class AC{constructor(){return off;}}
  const AP=Object.getPrototypeOf(off.createGain().gain);AP.setTargetAtTime=function(v,t,tc){this.linearRampToValueAtTime(v,t+Math.max(0.005,tc));};   // offline library: setTargetAtTime unstable
  const E=loadEngine({AudioContext:AC});Object.assign(E.cfg,{surface,profile:false,driven:true,sound:true,auto:true});E.selectTrack('rundkurs');
  E.audioInit();const A=E.AU;if(!A){console.log('audio init failed');return;}
  let T=0;const real=A.ctx;
  A.ctx={get currentTime(){return T;},state:'running',resume(){},createBufferSource:()=>real.createBufferSource(),createGain:()=>real.createGain()};
  const S=E.S;const log=[];
  for(let f=0;f<Math.floor(dur*60);f++){T=f/60;script(E,T);E.readInput(1/60);for(let k=0;k<6;k++)E.step(E.DT);E.audioUpdate();
    if(f%30===0)log.push(T.toFixed(1)+':'+S.rpm.toFixed(0)+'/g'+E.CAR.gears[S.gi].n+'/'+(Math.hypot(S.vx,S.vy)*3.6).toFixed(0)+'kmh/thr'+E.IN.thr.toFixed(1));}
  const buf=await real.startRendering();
  let peak=0,rms=0;const d=buf.getChannelData(0);for(let i=0;i<d.length;i++){peak=Math.max(peak,Math.abs(d[i]));rms+=d[i]*d[i];}rms=Math.sqrt(rms/d.length);
  wav(name+'.wav',buf);
  console.log(name,'peak',peak.toFixed(3),'rms',rms.toFixed(3),'nan',isNaN(rms));console.log('  '+log.join(' '));
}
(async()=>{
  await demo('sound_tarmac','tarmac',(E,t)=>{E.keys['arrowup']=(t>1&&t<7)||(t>9&&t<11);E.keys['arrowdown']=t>7.5&&t<9;E.keys['arrowleft']=t>9.5&&t<11;},12);
  await demo('sound_gravel','gravel',(E,t)=>{E.keys['arrowup']=(t>1&&t<6)||(t>7.5&&t<10);E.keys['arrowleft']=t>6&&t<8;E.keys[' ']=t>6.2&&t<6.8;},11);
})();
// Pegelverlauf in 0,5-s-Fenstern
function levels(path){const b=fs.readFileSync(path);const n=(b.length-44)/4;const sr=44100;const win=Math.floor(sr*0.5);const out=[];
  for(let w=0;w*win<n;w++){let s=0,pk=0,c=0;for(let i=w*win;i<Math.min(n,(w+1)*win);i++){const v=b.readInt16LE(44+i*4)/32768;s+=v*v;pk=Math.max(pk,Math.abs(v));c++;}out.push((20*Math.log10(Math.sqrt(s/c)+1e-9)).toFixed(0)+'/'+(20*Math.log10(pk+1e-9)).toFixed(0));}
  return out.join(' ');}
setTimeout(()=>{for(const n of ['sound_tarmac','sound_gravel'])console.log(n,'dBFS rms/peak je 0,5 s:',levels(n+'.wav'));},100);
