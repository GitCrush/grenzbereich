const {loadEngine}=require('./load');
function time(lab,mod){const E=loadEngine({render:true,w:1600,h:900});Object.assign(E.cfg,{view:'driver',surface:'gravel',mouse:false});E.selectTrack('rundkurs');E.resize();mod&&mod(E);
 const S=E.S,T=E.TRACK;let i=0;while(T.pts[i].s<520)i++;const p=T.pts[i],q=E.LINE[i];S.x=q.x;S.y=q.y;S.psi=Math.atan2(p.ty,p.tx);S.idx=i;S.vx=80/3.6;const w=S.vx/E.CAR.Rw;S.w=[w,w,w,w];S.gi=4;S.rpm=4500;S.we=471;E.vertInit();
 for(let k=0;k<30;k++)E.step(E.DT);E.draw();const t0=process.hrtime.bigint();for(let k=0;k<30;k++){E.IN.thr=0.5;for(let q=0;q<6;q++)E.step(E.DT);E.draw();}console.log(lab.padEnd(30),(Number(process.hrtime.bigint()-t0)/1e6/30).toFixed(1),'ms/frame');}
time('full day');
time('no near field',E=>{E.cfg.near=false;});
time('no scenery (simple)',E=>{E.cfg.scenery=false;E.cfg.daylight=false;});
time('no profile (flat)',E=>{E.cfg.profile=false;E.selectTrack('rundkurs');});
time('no vegetation',E=>{E.TRACK.pts.bush.length=0;});
time('no far field',E=>{E.cfg.far=false;});
