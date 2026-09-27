// Headless screenshots of the driver or top view.
// Usage: node shot.js '[["name",sPos,kmh,{"surface":"gravel","top":true,"steer":0.1}]]'
const {loadEngine}=require('./load');const fs=require('fs');
function shot(name,sPos,vk,opts={}){const E=loadEngine({render:true,w:opts.w||1600,h:opts.h||900});Object.assign(E.cfg,{view:'driver',mouse:false,surface:opts.surface||'tarmac',profile:opts.profile!==false,driven:opts.driven!==false});E.selectTrack(opts.track||'rundkurs');E.resize();
  const T=E.TRACK;let i=0;while(T.pts[i].s<sPos)i++;const p=T.pts[i];const S=E.S;const lat=opts.lat!==undefined?opts.lat:(E.LINE?E.LINE[i].o:0);
  S.x=p.x+lat*p.nx;S.y=p.y+lat*p.ny;S.psi=Math.atan2(p.ty,p.tx);S.idx=i;S.vx=vk/3.6;const w=S.vx/E.CAR.Rw;S.w=[w,w,w,w];S.gi=4;S.rpm=S.vx/E.CAR.Rw*E.CAR.gears[4].r*E.CAR.final*9.55;S.we=S.rpm*0.10472;E.vertInit();
  for(let k=0;k<(opts.steps||60);k++){E.IN.steer=opts.steer||0;E.IN.thr=opts.thr!==undefined?opts.thr:0.3;E.step(E.DT);}
  if(opts.top){E.cfg.view='top';for(let q=0;q<80;q++)E.drawTopDown();}else{E.draw();for(let k=0;k<6;k++)E.step(E.DT);E.draw();}
  fs.writeFileSync(name+'.png',E.canvases.world.toBuffer('image/png'));
  console.log(name,'s',sPos,'z',S.zRoad.toFixed(1),'grade',(S.slopeX*100).toFixed(1)+'%','load factor',(S.gApp/9.81).toFixed(2));}
for(const j of JSON.parse(process.argv[2]))shot(...j);
