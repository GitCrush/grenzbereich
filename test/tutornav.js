// 1) A driver who steers ONLY by the tutor's recommendation (0.15 s late, 600 deg/s hand), pedals from the phantom:
//    does he get round, and how close to the phantom's time?  2) Skidpad: silent on the line and on a tighter radius,
//    catches throttle stabs.
const {loadEngine}=require('./load');const DEG=180/Math.PI;const clamp=(v,a,b)=>v<a?a:v>b?b:v;const f=(x,d=1)=>(+x).toFixed(d);
function lap(track,surface,drive,mode,pace){const E=loadEngine();if(pace){E.GAI.aLat=pace;E.GAI.aBrk=pace;}Object.assign(E.cfg,{surface,drive,abs:false,auto:true,steer:'direct',uniform:true,profile:true,driven:true,tutor:true});E.VERT.amp=1;E.applyDriveSetup(drive);E.selectTrack(track);const S=E.S,T=E.TRACK,CAR=E.CAR;
  let t=0,laps=0,prevS=T.pts[S.idx].s,tLap=null,off=0,sw=0,brk=0;const q=[];
  while(t<320&&laps<2){const g=E.ghostCommand(brk,E.DT);brk=g.brk;
    if(mode==='ghost'){E.IN.steer=g.steer;E.IN.thr=g.thr;E.IN.brk=g.brk;}
    else{if(Math.round(t/E.DT)%6===0){const tu=E.tutorAdvice();q.push({t,tg:tu?tu.target:S.delta});}
      while(q.length>1&&q[1].t<=t-0.15)q.shift();const d=q.length?q[0].tg:0;
      const rate=600/DEG/CAR.ratio*E.DT;sw+=clamp(d-sw,-rate,rate);E.IN.steer=clamp(sw/CAR.maxSteer,-1,1);E.IN.thr=g.thr;E.IN.brk=g.brk;}
    E.step(E.DT);t+=E.DT;const s=T.pts[S.idx].s;if(s<prevS-800){laps++;if(laps===1)tLap=t;if(laps===2)tLap=t-tLap;}prevS=s;if(laps>=1&&!S.onTrack)off+=E.DT;}
  return laps>=2?f(tLap)+' s (off '+f(off)+')':'did not finish';}
console.log('steering only from the tutor (0.15 s late), pedals from the phantom speed plan at 50 % pace (the plan is made for the ideal line, the follower keeps his own):');
for(const [track,surface,drive] of [['rundkurs','tarmac','awd'],['rundkurs','gravel','awd'],['rundkurs','gravel','rwd'],['etappe','tarmac','awd'],['etappe','gravel','awd']])
  console.log('  '+(track+' '+surface+' '+drive).padEnd(22),'phantom',lap(track,surface,drive,'ghost',0.5).padEnd(20),'| tutor follower',lap(track,surface,drive,'tutor',0.5));
