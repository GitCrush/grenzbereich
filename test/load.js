// Headless loader: runs engine.js in a vm context with DOM stubs.
const vm=require('vm'), fs=require('fs');

function stub(){
  const f=function(){};
  return new Proxy(f,{
    get(t,p){
      if(p===Symbol.toPrimitive)return ()=>0;
      if(p===Symbol.iterator)return ()=>[][Symbol.iterator]();
      if(p==='then')return undefined;
      if(p==='length'||p==='width'||p==='height'||p==='clientWidth'||p==='clientHeight')return 0;
      if(p==='children'||p==='childNodes')return [];
      if(p==='textContent'||p==='innerHTML'||p==='id'||p==='className')return '';
      if(p==='classList')return {add(){},remove(){},toggle(){},contains(c){return c==='mobile'&&!!globalThis.__mobile;}};
      if(p==='style')return {};
      if(p==='dataset')return {};
      return stub();
    },
    set(){return true;},
    has(){return false;},
    apply(){return stub();},
    construct(){return stub();}
  });
}

function loadEngine(opts){
  opts=opts||{};
  let canvases={};
  if(opts.render){
    const C=require('@napi-rs/canvas');
    const mk=(w,h)=>{const c=C.createCanvas(w,h);c.clientWidth=w;c.clientHeight=h;c.addEventListener=()=>{};c.classList={add(){},remove(){},toggle(){},contains(){return false}};c.style={};c.getBoundingClientRect=()=>({left:0,top:0,width:w,height:h});c.requestPointerLock=()=>{};return c;};
    canvases={world:mk(opts.w||1280,opts.h||720),mapc:mk(274,150),tires:mk(150,112),gg:mk(112,112),steerc:mk(200,60)};
    opts.Path2D=C.Path2D;
  }
  const doc=stub();
  const winBase=stub();
  const winProxy=new Proxy(winBase,{get(t,p){ if(p==='AudioContext')return opts.AudioContext; if(p==='webkitAudioContext')return undefined; if(p==='ResizeObserver')return stub(); return t[p]; }, has(){return false;}});
  const docProxy=new Proxy(doc,{get(t,p){ if(p==='getElementById')return id=>canvases[id]||stub(); return t[p]; }});
  const ctx={
    console, Math, Object, Array, Float64Array, Float32Array, Uint8Array, Number, String, Boolean, JSON, Date, Symbol, Proxy, Promise, Error, Map, Set,
    document:docProxy, window:winProxy, navigator:stub(), screen:stub(), performance:{now:()=>0},
    requestAnimationFrame(){}, addEventListener(){}, removeEventListener(){}, setTimeout(){}, clearTimeout(){},
    Path2D:opts.Path2D||stub(), DOMPoint:class{constructor(x,y){this.x=x;this.y=y;}}, KeyboardEvent:stub(), ResizeObserver:stub(), devicePixelRatio:1,
    isFinite, isNaN, parseFloat, parseInt, Infinity, NaN, undefined, innerWidth:1280, innerHeight:720,
  };
  ctx.globalThis=ctx; ctx.self=ctx;
  vm.createContext(ctx);
  const src=fs.readFileSync(__dirname+'/'+(process.env.ENGINE||'engine.js'),'utf8');
  vm.runInContext(src,ctx,{filename:'engine.js'});
  ctx.__sim.canvases=canvases;
  return ctx.__sim;
}
module.exports={loadEngine};
