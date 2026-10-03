import sys
src,dst=sys.argv[1],sys.argv[2]
s=open(src,encoding='utf8').read()
def rep(old,new):
    global s
    if old not in s: print('MISSING',old[:70]); return
    s=s.replace(old,new,1)
rep("function fillCam(P,style","var __TAG='',__REC=null;function __rec(C){if(!__REC||!__TAG)return;let x0=1e9,x1=-1e9,y0=1e9,y1=-1e9,a=0;const Q=C.map(c=>proj(c));for(let i=0;i<Q.length;i++){const p=Q[i],q=Q[(i+1)%Q.length];a+=p[0]*q[1]-q[0]*p[1];x0=Math.min(x0,p[0]);x1=Math.max(x1,p[0]);y0=Math.min(y0,p[1]);y1=Math.max(y1,p[1]);}const o=__REC.get(__TAG)||{a:0,x0:1e9,x1:-1e9,y0:1e9,y1:-1e9};o.a+=Math.abs(a)/2;o.x0=Math.min(o.x0,x0);o.x1=Math.max(o.x1,x1);o.y0=Math.min(o.y0,y0);o.y1=Math.max(o.y1,y1);__REC.set(__TAG,o);}\nfunction fillCam(P,style")
rep("const C=clipPoly(P); if(C.length<3)return;\n  cx.fillStyle=style;","const C=clipPoly(P); if(C.length<3)return;__rec(C);\n  cx.fillStyle=style;")
rep("const band=(r0,r1,z0a,z0b,z1a,z1b,fd)=>{","const band=(r0,r1,z0a,z0b,z1a,z1b,fd)=>{__TAG='band:'+(a.i0>>2)+':'+side;")
rep("      if(!withVerge)continue;","      if(!withVerge)continue;__TAG='verge:'+(a.i0>>2)+':'+side;")
rep("        fillCam(outer.concat(inner.reverse()),col);","        __TAG='far:'+side;fillCam(outer.concat(inner.reverse()),col);__TAG='forest:'+side;")
rep("    if(TRACK.hasZ)terrainSeg(a,b,ax,ay,bx,by,dSeg,winFade(k),true);","    if(TRACK.hasZ)terrainSeg(a,b,ax,ay,bx,by,dSeg,winFade(k),true);__TAG='road:'+(a.i0>>2);")
rep("      if(!(P[0].z<NEAR&&P[1].z<NEAR&&P[2].z<NEAR&&P[3].z<NEAR)){let rc=hazed(sur.fill3,SC,d);","      __TAG='road:'+(a.i0>>2);if(!(P[0].z<NEAR&&P[1].z<NEAR&&P[2].z<NEAR&&P[3].z<NEAR)){let rc=hazed(sur.fill3,SC,d);")
rep("function selectTrack(id){\n  trackId=id; TRACK=buildTrack(TRACKS[id]);","function selectTrack(id){\n  trackId=id; TRACK=buildTrack(TRACKS[id]);TRACK.pts.forEach((p,i)=>p.i0=i);")
rep("rollBars:typeof rollBars","__setRec:(m)=>{__REC=m;__TAG='';}, rollBars:typeof rollBars")
open(dst,'w',encoding='utf8').write(s)
