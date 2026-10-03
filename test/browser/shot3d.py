import asyncio, sys
from playwright.async_api import async_playwright
import os
URL='file://'+os.path.abspath(os.path.join(os.path.dirname(__file__),'../../index.html'))
async def main(track,surface,secs,out,w=1280,h=720):
    async with async_playwright() as p:
        b=await p.chromium.launch(args=['--use-gl=swiftshader','--enable-unsafe-swiftshader','--ignore-gpu-blocklist'])
        ctx=await b.new_context(viewport={'width':w,'height':h},ignore_https_errors=True)
        pg=await ctx.new_page()
        errs=[];pg.on('pageerror',lambda e:errs.append(str(e)));pg.on('console',lambda m:errs.append(m.text) if m.type=='error' else None)
        await pg.goto(URL);await pg.wait_for_timeout(1500)
        ok=await pg.evaluate("()=>typeof THREE!=='undefined'")
        await pg.evaluate(f"()=>{{document.getElementById('trackmenu').classList.remove('on');cfg.surface='{surface}';selectTrack('{track}');window.readInput=function(){{ghostAI();}};}}")
        await pg.wait_for_timeout(int(secs*1000))
        info=await pg.evaluate("()=>({gl:glActive(),ok:GL.ok,objs:(GL.objs||[]).length,fms:fms})")
        await pg.screenshot(path=out)
        await b.close()
        print(track,surface,'THREE',ok,info,'errors',errs[:3])
asyncio.run(main(sys.argv[1],sys.argv[2],float(sys.argv[3]),sys.argv[4]))
