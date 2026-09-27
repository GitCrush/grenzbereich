// Browser screenshots with the full interface: side panels, HUD, overlays.
// The car is driven by the phantom's driver model, so the images show the
// physics at the limit rather than a posed state. Needs Playwright:
//   npm install playwright && npx playwright install chromium
// Usage: node uishot.js [outdir]   (default ../docs/screenshots)
// Frames are taken at wall-clock times after the start, so two runs give
// similar, not identical, images.
const {chromium}=require('playwright');
const path=require('path');
const OUT=process.argv[2]||path.join(__dirname,'../docs/screenshots');
const PAGE='file://'+path.join(__dirname,'../index.html');
const SHOTS=[
  // name, surface, view, setup panel open, top-view camera mode, tyre forces, phantom, seconds after start
  {name:'driver-view-gravel',        surf:'gravel', view:'persp', setup:true,  t:23},
  {name:'top-view-gravel',           surf:'gravel', view:'top',   mode:1, forces:true, t:23},
  {name:'driver-view-banked-tarmac', surf:'tarmac', view:'persp', ghost:false, t:19},   // the phantom would sit on the car here
];
(async()=>{
  const browser=await chromium.launch();
  for(const s of SHOTS){
    const p=await browser.newPage({viewport:{width:1600,height:900},deviceScaleFactor:1});
    p.on('pageerror',e=>console.error(s.name,String(e)));
    await p.goto(PAGE);
    await p.waitForTimeout(800);
    await p.mouse.move(1450,880);                 // pointer over the side panel, off the view
    await p.keyboard.press('Escape');             // close the track menu
    await p.evaluate(s=>{
      const c=q=>document.querySelector(q).click();
      c(`#segSurf [data-v=${s.surf}]`);
      if(s.view!=='persp')c(`#segView [data-v=${s.view}]`);
      c('#segMouse [data-v=mouse]');              // mouse off: no steering hint over the image
      if(s.forces)c('#segViz [data-v=forces]');
      document.getElementById('setup').classList.toggle('on',!!s.setup);
      reset();
      if(s.mode)cam.mode=s.mode;
      readInput=function(){ghostAI();};           // the phantom's driver model drives the player car
    },s);
    await p.waitForTimeout(2500); if(s.ghost!==false)await p.keyboard.press('g');   // phantom, 2.5 s behind
    await p.waitForTimeout((s.t-2.5)*1000);
    await p.screenshot({path:path.join(OUT,s.name+'.png')});
    console.log(s.name);
    await p.close();
  }
  await browser.close();
})();
