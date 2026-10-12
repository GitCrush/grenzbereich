// Validate a ring drift controller (NS 5 with prediction, or NS 7 augmented) at REACT: establish with the angle law until TSW,
// then hold β target and e 0 (ramped from where the car is over ERAMP s). WASH=1: v and δ references washed out (2 s means).
const L=require('./dclib'),CT=require('./ctl');const {f}=L;const env=process.env;
const M=JSON.parse(require('fs').readFileSync(env.MODEL||__dirname+'/model.json'));const n=M.Ad.length;
const R=M.R,react=env.REACT!==undefined?+env.REACT:(M.react||0);const o=L.setup({smooth:env.SMOOTH==='1',R,drive:env.DRIVE||M.drive,surf:env.SURF,NS:n,react});const {E,S,C,d}=o,DT=E.DT;if(n>=6)C.drvLead=0;
const K=CT.mk(M,C,DT);const TSW=+(env.TSW||3),ERAMP=+(env.ERAMP||6),BREF=d*(env.BREF!==undefined?+env.BREF:M.BT*57.3)/57.3;
let t=0,k=0,I=0,e0=null,cmd=0,thr=0,vL=null,dL=null,spin=false;const sb=[],se=[];
while(t<(+env.T||30)){const x=L.obs(o);
  if(k%6===0){const kC=L.aimK(o),thf=L.thF(o);
    if(t<Math.min(1,TSW)){cmd=thf+0.3*(x[2]*kC-S.r);thr=0.2;}else if(t<Math.min(3,TSW)){cmd=thf+0.3*(x[2]*kC-S.r);thr=M.Tff;}
    else if(t<TSW){cmd=thf+0.3*(x[2]*kC-S.r)+4*(x[0]-BREF);I=Math.max(-0.5,Math.min(0.5,I+0.15*(BREF-x[0])*d/60));thr=M.Tff+I;}
    else{if(e0===null){e0=x[3];vL=x[2];dL=S.delta;}vL+=(x[2]-vL)/120;dL+=(S.delta-dL)/120;const er=e0*(1-Math.min(1,(t-TSW)/ERAMP));
      const r=env.WASH==='1'?{b:BREF,e:er,h:0,r:Math.sign(M.x0[1])*x[2]/R,v:vL,d:dL}:{b:BREF,e:er};[cmd,thr]=K.step(x,S.dLP,r);}
    if(env.TRACE&&k%(+env.TRACE)===0)console.log(f(t,1),'β',f(x[0]*57.3,1),'r',f(x[1]*57.3,1),'v',f(x[2]*3.6,0),'e',f(x[3],1),'h',f(x[4]*57.3,1),'δ',f(S.delta*57.3,1),'cmd',f(cmd*57.3,1),'thr',f(thr));}
  E.IN.steer=Math.max(-1,Math.min(1,cmd/C.maxSteer));E.IN.thr=thr;E.IN.brk=0;E.IN.hb=0;E.step(DT);t+=DT;k++;
  if(t>(+env.STAT||12)){sb.push(Math.abs(x[0])*57.3);se.push(x[3]);}
  if(Math.abs(x[0])>1.2||Math.abs(x[3])>28){spin=true;break;}}
const m=a=>a.reduce((x,y)=>x+y,0)/Math.max(1,a.length),sd=a=>{const mm=m(a);return Math.sqrt(m(a.map(x=>(x-mm)**2)));};
console.log(`  R${R} ${E.cfg.drive} r${react} NS${n}${env.WASH==='1'?' wash':''} | β ${f(m(sb),1)}±${f(sd(sb),1)} (ref ${f(Math.abs(BREF)*57.3,0)}) | e ${f(m(se),1)}±${f(sd(se),1)} | v ${f(Math.hypot(S.vx,S.vy)*3.6,0)}`+(spin?' | SPIN/OUT at '+f(t,1):'')+` | zb ${f(K.z[0],2)} ze ${f(K.z[1],1)}`);
