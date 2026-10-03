import asyncio, sys, base64
from playwright.async_api import async_playwright
import os
URL="file://"+os.path.abspath(os.path.join(os.path.dirname(__file__),"../../index.html"))
async def main(track,surface,t0,out):
    async with async_playwright() as p:
        b=await p.chromium.launch(args=['--use-gl=swiftshader','--enable-unsafe-swiftshader'])
        pg=await b.new_page(viewport={'width':960,'height':540})
        await pg.goto(URL);await pg.wait_for_timeout(600)
        await pg.evaluate(f"""()=>{{document.getElementById('trackmenu').classList.remove('on');cfg.surface='{surface}';selectTrack('{track}');window.readInput=function(){{ghostAI();}};
          window.__frames=[];window.__rec=false;const cv=document.querySelector('canvas');const od=window.draw;
          window.draw=function(){{od();if(window.__rec&&window.__frames.length<24)window.__frames.push(cv.toDataURL('image/jpeg',0.8));}};}}""")
        await pg.wait_for_timeout(int(t0*1000))
        await pg.evaluate("()=>{window.__rec=true;}")
        await pg.wait_for_timeout(1500)
        fr=await pg.evaluate("()=>window.__frames")
        await b.close()
    from PIL import Image
    import io
    ims=[Image.open(io.BytesIO(base64.b64decode(f.split(',')[1]))) for f in fr[:24]]
    w,h=ims[0].size;c=Image.new('RGB',(w*2,h*3))
    for i,im in enumerate(ims[:6]):c.paste(im.resize((w,h)),((i%2)*w,(i//2)*h))
    c.save(out+'_a.png')
    print('frames',len(fr))
    for i,im in enumerate(ims):im.save(f'{out}_{i:02d}.jpg')
asyncio.run(main(sys.argv[1],sys.argv[2],float(sys.argv[3]),sys.argv[4]))
