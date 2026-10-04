const {loadEngine}=require('./load');const f=(x,d=1)=>(+x).toFixed(d);
const [c,e,sd]=process.argv.slice(2).map(Number);
const E=loadEngine();Object.assign(E.cfg,{surface:'gravel',abs:false,auto:true,steer:'direct',uniform:true,profile:true,driven:true});E.VERT.amp=1;
const d=E.randomTrackDef(sd,c,e);E.TRACKS.random=d;E.selectTrack('random');const S=E.S,T=E.TRACK;let t=0,prevS=T.pts[S.idx].s,done=false,off=0,air=0;
while(t<240&&!done){E.ghostAI();E.step(E.DT);t+=E.DT;const s=T.pts[S.idx].s;if(s<prevS-T.pts[T.N-1].s/2)done=true;prevS=s;if(!S.onTrack)off+=E.DT;if(S.gApp<0.3*9.81)air+=E.DT;}
console.log('curves',c,'elev',e,'seed',sd,'|',d.desc,'| phantom',done?f(t)+' s, off '+f(off)+' s, light on the wheels '+f(air,1)+' s':'did not finish');
