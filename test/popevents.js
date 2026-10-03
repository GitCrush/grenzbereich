// Pop / build-up events: an element (keyed per 4 track points) that appears with > 300 px², or an existing one whose
// screen area jumps by > 2500 px² and > 40 % in one frame – without entering from the screen edge.
const {loadEngine}=require('./load');const clamp=(v,a,b)=>v<a?a:v>b?b:v;
function run(surface,track,s0,secs,detailFlip){const E=loadEngine({render:true,w:960,h:540});Object.assign(E.cfg,{view:'driver',surface,mouse:false});E.selectTrack(track);E.resize();
 const S=E.S,T=E.TRACK;let i=0;while(T.pts[i].s<s0)i++;const p=T.pts[i],q=E.LINE[i];S.x=q.x;S.y=q.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=i;const vT=95/3.6;S.vx=vT;const w=vT/E.CAR.Rw;S.w=[w,w,w,w];S.gi=4;S.rpm=4500;S.we=471;E.vertInit();const c={I:0};
 let prev=null;const ev={};const ex=[];
 for(let f=0;f<60*secs;f++){if(detailFlip){E.DETAIL.mode='low';E.DETAIL.lvl=(Math.floor(f/90)%2);}
  for(let k=0;k<6;k++){const e=vT-Math.hypot(S.vx,S.vy);c.I=clamp(c.I+e*E.DT*0.6,-0.9,0.9);const u=0.35*e+c.I;E.IN.thr=clamp(u,0,1);E.IN.brk=clamp(-u*0.5,0,1);E.IN.steer=E.lineSteer().delta/E.CAR.maxSteer;E.step(E.DT);}
  const rec=new Map();E.__setRec(rec);E.draw();E.__setRec(null);
  if(prev){for(const [k,o] of rec){if(o.x0<2||o.x1>958||o.y1>538)continue;const pv=prev.get(k);const type=k.split(':')[0];
    if(!pv&&o.a>300){ev[type+' new']=(ev[type+' new']||0)+1;if(ex.length<5)ex.push(k+' new '+o.a.toFixed(0));}
    else if(pv&&o.a-pv.a>2500&&o.a>1.4*pv.a){ev[type+' grows']=(ev[type+' grows']||0)+1;if(ex.length<5)ex.push(k+' '+pv.a.toFixed(0)+'->'+o.a.toFixed(0));}}}
  prev=rec;}
 console.log((surface+' '+track+(detailFlip?' (detail toggling)':'')).padEnd(36),JSON.stringify(ev),ex.length?'e.g. '+ex.join('; '):'');}
const a=process.argv.slice(2);run(a[0],a[1],+a[2],+a[3],a[4]==='flip');
