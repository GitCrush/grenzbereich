import asyncio
from playwright.async_api import async_playwright
CHK=r"""()=>{const pick=(e)=>{const r=e.getBoundingClientRect();const t=document.elementFromPoint(r.left+r.width/2,r.top+r.height/2);return t===e||e.contains(t)?'clickable':'covered by '+(t?t.tagName+'#'+t.id:'nothing');};
  return {setupOpen:document.getElementById('setup').classList.contains('on'),tab:pick(document.getElementById('setupTab')),template:pick(document.querySelector('#segTpl button[data-v="Snow"]')),slider:pick(document.getElementById('rLsd')),gl:document.getElementById('world3d').style.display};}"""
async def main():
    async with async_playwright() as p:
        b=await p.chromium.launch(args=['--use-gl=swiftshader','--enable-unsafe-swiftshader','--ignore-gpu-blocklist'])
        ctx=await b.new_context(viewport={'width':1280,'height':720},ignore_https_errors=True);pg=await ctx.new_page()
        await pg.goto('file://'+__import__('os').path.abspath('../../index.html')+'');await pg.wait_for_timeout(1200)
        await pg.evaluate("()=>{document.getElementById('trackmenu').classList.remove('on');selectTrack('rundkurs');}");await pg.wait_for_timeout(1500)
        await pg.click('#setupTab');await pg.wait_for_timeout(600)
        print(await pg.evaluate(CHK))
        await pg.click('#segTpl button[data-v="Snow"]');await pg.wait_for_timeout(300)
        print('real click on template Snow worked:',await pg.evaluate("()=>document.querySelector('#segTpl button[data-v=\"Snow\"]').classList.contains('on')"))
        await pg.screenshot(path='setup_open.png');await b.close()
asyncio.run(main())
