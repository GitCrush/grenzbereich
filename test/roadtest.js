const {loadEngine}=require('./load');const DEG=180/Math.PI,G=9.81;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=2)=>(+x).toFixed(d);
function lap(profile,surface,opts={}){const E=loadEngine();Object.assign(E.cfg,{surface,abs:false,auto:true,steer:'direct',uniform:true,profile});E.VERT.amp=opts.rough===undefined?0:opts.rough;if(opts.aLat)E.GAI.aLat=opts.aLat;E.selectTrack('rundkurs');const S=E.S,T=E.TRACK;
  const rec=[];let t=0,laps=0,prevS=T.pts[S.idx].s,s0=prevS,started=false,tLap=null;
  while(t<240){E.ghostAI();E.step(E.DT);t+=E.DT;const s=T.pts[S.idx].s;if(s<prevS-800){laps++;if(laps===1)tLap=t;if(laps===2){tLap=t-tLap;break;}}prevS=s;
    if(laps>=1)rec.push({s,t,Fz:S.Fz.reduce((a,b)=>a+b,0)/(E.CAR.m*G),gA:S.gApp/G,ay:S.ay/G,thr:E.IN.thr,brk:E.IN.brk,use:Math.max(...S.slip),v:Math.hypot(S.vx,S.vy)*3.6,on:S.onTrack,dF:(S.Fz[2]+S.Fz[3]-S.Fz[0]-S.Fz[1]),beta:Math.atan2(S.vy,Math.abs(S.vx))*DEG});}
  return {rec,len:T.len,tLap,off:rec.filter(r=>!r.on).length*E.DT,vmax:Math.max(...rec.map(r=>r.v)),vmin:Math.min(...rec.map(r=>r.v))};}
function at(rec,s){let best=null,bd=1e9;for(const r of rec){const d=Math.abs(r.s-s);if(d<bd){bd=d;best=r;}}return bd<3?best:null;}
for(const surface of ['tarmac','gravel']){
  const F=lap(false,surface),P=lap(true,surface);
  console.log('=== Phantom driver, flying lap, '+surface+' ===  lap time flat '+f(F.tLap,1)+' s / profile '+f(P.tLap,1)+' s   off track flat '+f(F.off,1)+' s / profile '+f(P.off,1)+' s   v max '+f(F.vmax,0)+'/'+f(P.vmax,0));
  console.log('location                  s  | flat: v     ΣFz/mg  ay    use  β   | profile: v   ΣFz/mg gApp  ay    use  β    Fz r−f');
  for(const [lab,s] of [['climb 5.5 %',535],['crest',770],['descent 6.4 %',989],['dip/compression',1020],['banked 5.2° left',1190],['crest before tight right',1345],['off-camber right',208]]){
    const a=at(F.rec,s),b=at(P.rec,s);if(!a||!b){console.log(lab,'–');continue;}
    console.log(lab.padEnd(24),String(s).padStart(5),' |',f(a.v,0).padStart(4),' ',f(a.Fz).padStart(5),' ',f(a.ay).padStart(5),' ',f(a.use),' ',f(a.beta,0).padStart(3),' |',f(b.v,0).padStart(4),' ',f(b.Fz).padStart(5),' ',f(b.gA),' ',f(b.ay).padStart(5),' ',f(b.use),' ',f(b.beta,0).padStart(3),' ',f(b.dF,0).padStart(6),' N');
  }
}
