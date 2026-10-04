import sys
src=sys.argv[1] if len(sys.argv)>1 else '../index.html'
h=open(src,encoding='utf8').read()
i=h.rfind('<script>'); j=h.rfind('</script>')
js=h[i+len('<script>'):j].rstrip()
assert js.endswith("requestAnimationFrame(loop);")
js=js[:-len("requestAnimationFrame(loop);")]
js+="""
globalThis.__sim={
  get S(){return S}, set S(v){S=v}, get IN(){return IN}, set IN(v){IN=v}, get VERT(){return VERT},
  cfg,CAR,SURF,OFF,WHEELS,TRACKS,PED,step,reset,selectTrack,vertInit,mf,engineTorque,shift,autoShift,
  get TRACK(){return TRACK}, lap, keys, DT, readInput, get frameN(){return frameN}, ghostAI, spawnGhost, get LINE(){return LINE}, GAI, ROAD, applyDriveSetup, DRIVE_SETUP, applySetupTemplate, SETUP_TEMPLATES, TOUCH, touchInput, tutorAdvice, lineSteer, lineSpeed, ghostCommand, randomTrackDef, TRACKS, RAND, SURFX, BIOMES, genHazards, hazardAt, ensureHazards, genPaceNotes, set TUT_LEAD(v){TUT_LEAD=v}, get TUT_LEAD(){return TUT_LEAD}, sceneFor, hazed, vary, mixCol, dsp, set RDBG(v){RDBG=v}, trackZ, nearestIdx, WHEELS, setupCam, DETAIL, groundHeight, nearestIdxWide, landClearance, get TUT_BIAS(){return TUT_BIAS}, collectGroundMarks, drawRoadPass, drawBushP, SURF, stepGhost, get ghost(){return ghost}, audioInit, audioUpdate, get AU(){return AU}, set AU(v){AU=v}, draw, drawPerspective, drawTopDown, resize, cam3, VIEW, rollBars:typeof rollBars==='function'?rollBars:null
};
"""
open('engine.js','w',encoding='utf8').write(js)
print('engine.js',len(js.splitlines()),'lines')
