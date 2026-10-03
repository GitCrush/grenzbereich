import asyncio, json, sys
from playwright.async_api import async_playwright
import os
URL="file://"+os.path.abspath(os.path.join(os.path.dirname(__file__),"../../index.html"))
async def main(track,surface,secs,out,mode='keys'):
    async with async_playwright() as p:
        b=await p.chromium.launch(args=['--use-gl=swiftshader','--enable-unsafe-swiftshader'])
        pg=await b.new_page(viewport={'width':1280,'height':720})
        errs=[];pg.on('pageerror',lambda e:errs.append(str(e)));pg.on('console',lambda m:errs.append('console:'+m.text) if m.type in('error','warning') else None)
        await pg.goto(URL);await pg.wait_for_timeout(800)
        await pg.evaluate(f"""()=>{{document.getElementById('trackmenu').classList.remove('on');cfg.surface='{surface}';selectTrack('{track}');
          window.__nullHz=0;const oh=window.horizonLine;window.horizonLine=function(W){{const r=oh(W);if(!r)window.__nullHz++;return r;}};
          window.__probe=[];const cv=document.querySelector('canvas#world')||document.querySelector('canvas');const c2=cv.getContext('2d');
          window.__shots=[];const pc=document.createElement('canvas');const pcx=pc.getContext('2d');
          const od=window.draw;window.draw=function(){{od();const W=cv.width,H=cv.height;
            const s=(x,y)=>{{const d=c2.getImageData(Math.floor(x*W),Math.floor(y*H),1,1).data;return d[0]+d[1]+d[2];}};
            const v=[s(.5,.04),s(.2,.1),s(.8,.1),s(.1,.42),s(.9,.42),s(.3,.47),s(.7,.47),fms];const pv=window.__probe[window.__probe.length-1];window.__probe.push(v);
            if(pv&&(Math.abs(v[0]-pv[0])>90||Math.abs(v[3]-pv[3])>150||Math.abs(v[4]-pv[4])>150||Math.abs(v[1]-pv[1])>150)&&window.__shots.length<8){{window.__shots.push([pc.toDataURL('image/png'),cv.toDataURL('image/png'),S.idx,S.psi,cam3.ex,cam3.ey,cam3.ez,JSON.stringify({{r:S.r,vx:S.vx,vy:S.vy}})]);}}
            pc.width=W;pc.height=H;pcx.drawImage(cv,0,0);}};}}""")
        if mode=='ghost':
            await pg.evaluate("()=>{window.readInput=function(){ghostAI();};}")
        else:
            await pg.keyboard.down('ArrowUp')
        for k in range(int(secs*4)):
            await pg.wait_for_timeout(250)
            if mode!='ghost' and k%8==4: await pg.keyboard.down('ArrowLeft')
            if mode!='ghost' and k%8==6: await pg.keyboard.up('ArrowLeft')
            if k==int(secs*2): await pg.screenshot(path=out+'_mid.png')
        r=await pg.evaluate("()=>({probe:window.__probe,nullHz:window.__nullHz,lvl:DETAIL.lvl,canvasW:document.querySelector('canvas').width,shots:window.__shots})")
        import base64
        for n,sh in enumerate(r['shots'][:8]):
            for m,tag in ((0,'prev'),(1,'cur')):
                open(f'{out}_jump{n}_{tag}.png','wb').write(base64.b64decode(sh[m].split(',')[1]))
            print('  jump',n,'idx',sh[2],'psi',round(sh[3],2),'cam',[round(x,1) for x in sh[4:7]],sh[7])
        r['shots']=None
        await b.close()
        json.dump({'r':r,'errs':errs[:20]},open(out+'.json','w'))
        pr=r['probe'];print(track,surface,'frames',len(pr),'nullHz',r['nullHz'],'errors',len(errs),errs[:3])
        import statistics
        for j,name in enumerate(['sky top','sky L','sky R','land L','land R','mid L','mid R']):
            jumps=sum(1 for a,c in zip(pr,pr[1:]) if abs(c[j]-a[j])>90)
            print(f'  {name:8} frame-to-frame jumps >90: {jumps}')
        print('  mean frame ms',round(statistics.mean(x[7] for x in pr),1))
asyncio.run(main(sys.argv[1],sys.argv[2],float(sys.argv[3]),sys.argv[4],sys.argv[5] if len(sys.argv)>5 else 'keys'))
