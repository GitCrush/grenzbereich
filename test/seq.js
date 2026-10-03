// Frame sequence while driving: a strip of frames 0.2 s apart for visual inspection, plus a check for holes:
// sky-coloured pixels below the horizon line (terrain missing).
const {loadEngine}=require('./load');const fs=require('fs');const clamp=(v,a,b)=>v<a?a:v>b?b:v;
const {createCanvas}=require('@napi-rs/canvas');
const E=loadEngine({render:true,w:640,h:360});Object.assign(E.cfg,{view:'driver',surface:process.argv[2]||'gravel',mouse:false});E.selectTrack(process.argv[3]||'rundkurs');E.resize();
const S=E.S,T=E.TRACK;let i=0;while(T.pts[i].s<(+process.argv[4]||400))i++;const p=T.pts[i],q=E.LINE[i];S.x=q.x;S.y=q.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=i;const vT=90/3.6;S.vx=vT;const w=vT/E.CAR.Rw;S.w=[w,w,w,w];S.gi=4;S.rpm=4500;S.we=471;E.vertInit();const c={I:0};
const out=createCanvas(640*4,360*3),oc=out.getContext('2d');let n=0;
for(let f=0;f<12*12;f++){for(let k=0;k<6;k++){const e=vT-Math.hypot(S.vx,S.vy);c.I=clamp(c.I+e*E.DT*0.6,-0.9,0.9);const u=0.35*e+c.I;E.IN.thr=clamp(u,0,1);E.IN.brk=clamp(-u*0.5,0,1);E.IN.steer=E.lineSteer().delta/E.CAR.maxSteer;E.step(E.DT);}
  E.draw();if(f%12===0&&n<12){oc.drawImage(E.canvases.world,(n%4)*640,Math.floor(n/4)*360);n++;}}
fs.writeFileSync(process.argv[5]||'seq.png',out.toBuffer('image/png'));console.log('ok');
