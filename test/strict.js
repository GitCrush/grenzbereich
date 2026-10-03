// Drive with a canvas that behaves like a browser's on bad input: ellipse/arc with a negative radius, gradients with
// non-finite coordinates and colour stops with invalid colours throw; invalid fillStyle/strokeStyle strings are ignored
// (the previous style stays). Any of these in the browser either aborts the frame or paints a surface in the wrong colour.
const {loadEngine}=require('./load');const clamp=(v,a,b)=>v<a?a:v>b?b:v;
const valid=c=>typeof c!=='string'||/^#[0-9a-f]{3,8}$/i.test(c)||/^rgba?\(\s*-?[\d.]+\s*,\s*-?[\d.]+\s*,\s*-?[\d.]+\s*(,\s*-?[\d.]+\s*)?\)$/.test(c);
function strictify(ctx,log){
  const fin=(...a)=>a.every(Number.isFinite);
  for(const m of ['ellipse','arc']){const o=ctx[m].bind(ctx);ctx[m]=function(...a){const r=m==='arc'?[a[2]]:[a[2],a[3]];if(r.some(x=>x<0))throw new Error(m+' negative radius '+r.join(','));return o(...a);};}
  const lg=ctx.createLinearGradient.bind(ctx),rg=ctx.createRadialGradient.bind(ctx);
  const wrapG=g=>{const ac=g.addColorStop.bind(g);g.addColorStop=(o,c)=>{if(!valid(c)||!Number.isFinite(o))throw new Error('addColorStop '+o+' '+c);return ac(o,c);};return g;};
  ctx.createLinearGradient=(...a)=>{if(!fin(...a))throw new Error('createLinearGradient non-finite '+a.join(','));return wrapG(lg(...a));};
  ctx.createRadialGradient=(...a)=>{if(!fin(...a))throw new Error('createRadialGradient non-finite '+a.join(','));if(a[2]<0||a[5]<0)throw new Error('radial negative');return wrapG(rg(...a));};
  for(const prop of ['fillStyle','strokeStyle']){const desc=Object.getOwnPropertyDescriptor(Object.getPrototypeOf(ctx),prop);
    Object.defineProperty(ctx,prop,{get(){return desc.get.call(ctx);},set(v){if(!valid(v)){log.bad[v]=(log.bad[v]||0)+1;return;}desc.set.call(ctx,v);}});}
}
function run(surface,track,secs,mode){const E=loadEngine({render:true,w:960,h:540});Object.assign(E.cfg,{view:'driver',surface,mouse:false});E.selectTrack(track);E.resize();
  const log={bad:{},err:{}};strictify(E.canvases.world.getContext('2d'),log);
  const S=E.S,CAR=E.CAR;let t=0;
  for(let f=0;f<60*secs;f++){for(let k=0;k<6;k++){if(mode==='ghost')E.ghostAI();else{E.IN.thr=0.8;E.IN.steer=Math.sin(t*0.9)*0.8;E.IN.hb=(Math.sin(t*0.37)>0.9)?1:0;}E.step(E.DT);t+=E.DT;}
    try{E.draw();}catch(e){const k=(e.message||String(e)).slice(0,90);if(!log.err[k]){log.err[k]={n:0,stack:(e.stack||'').split('\n').slice(1,3).join(' | ')};}log.err[k].n++;}}
  console.log(surface,track,mode,'| frames aborted:',Object.values(log.err).reduce((a,b)=>a+b.n,0),'| invalid colours:',JSON.stringify(log.bad).slice(0,300));
  for(const [k,v] of Object.entries(log.err))console.log('   ',v.n+'x',k,'\n      at',v.stack.slice(0,200));}
const a=process.argv.slice(2);run(a[0],a[1],+a[2],a[3]);
