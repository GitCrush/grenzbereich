import asyncio
from playwright.async_api import async_playwright
JS=r"""(gfx)=>{document.getElementById('trackmenu').classList.remove('on');setGfx(gfx);cfg.surface='gravel';selectTrack('etappe');window.readInput=function(){};
  const cv=document.getElementById('world'),c3=document.getElementById('world3d');const off=document.createElement('canvas');off.width=320;off.height=180;const oc=off.getContext('2d',{willReadFrequently:true});
  const grab=()=>{oc.fillStyle='#000';oc.fillRect(0,0,320,180);if(c3&&c3.style.display!=='none')oc.drawImage(c3,0,0,320,180);oc.drawImage(cv,0,0,320,180);return oc.getImageData(0,95,320,45).data.slice();};
  const res=[];for(const sp of [300,700,1100]){let i=0;while(TRACK.pts[i].s<sp)i++;const p=TRACK.pts[i];let A=null,B=null;
    for(const k of [0,1]){S.x=p.x+p.tx*0.5*k;S.y=p.y+p.ty*0.5*k;S.psi=Math.atan2(p.ty,p.tx);S.idx=i;S.vx=25;S.vy=0;S.r=0;S.zRoad=p.z;draw();draw();if(k===0)A=grab();else B=grab();}
    let m=0,sa=0;for(let j=0;j<A.length;j+=4){m+=Math.abs(A[j]-B[j])+Math.abs(A[j+1]-B[j+1])+Math.abs(A[j+2]-B[j+2]);sa+=A[j];}res.push([m/(A.length/4),sa/(A.length/4)]);}
  return res;}"""
async def run(path,gfx,label):
    async with async_playwright() as p:
        b=await p.chromium.launch(args=['--use-gl=swiftshader','--enable-unsafe-swiftshader','--ignore-gpu-blocklist'])
        ctx=await b.new_context(viewport={'width':1280,'height':720},ignore_https_errors=True);pg=await ctx.new_page()
        await pg.goto('file://'+path);await pg.wait_for_timeout(1200)
        r=await pg.evaluate(JS,gfx)
        await b.close();print(label.ljust(14),'image change per 0.5 m:',[round(x[0],1) for x in r],'mean',round(sum(x[0] for x in r)/len(r),1),'| mean brightness',[round(x[1]) for x in r])
async def main():
    await run(__import__('os').path.abspath('../../index.html'),'day','3D before')
    await run(__import__('os').path.abspath('../../index.html'),'day','3D now')
    await run(__import__('os').path.abspath('../../index.html'),'simple','2D (simple)')
asyncio.run(main())
