// Drift controllers for the ring experiments.
// mk(M, C, DT): NS 5 model → LQI on the road-wheel angle with a model prediction over the driver delay (the 10 Oct morning design);
//               NS 7 model with an augmented gain M.KA → LQI that knows the driver chain (delay buffer and lag as states).
// step(x, p, ref) → [cmd, thr]; ref = {b, r, v, e, h} (any missing → model's x0). p = S.dLP (the lagged hand target).
const L=require('./dclib');const ZB=+(process.env.ZB||0.5),ZE=+(process.env.ZE||40),SS=process.env.SS==='1';
function mk(M,C,DT){const n=M.Ad.length,ch=L.chain(C,DT);
  // a model identified at another reaction (other driver chain): redesign its gain for this chain (the vehicle part is the same car)
  if(M.KA&&M.KA[0].length!==n+1+ch.nd+2){M=Object.assign({},M,{KA:L.lqiAug(M,M.Qw||[1,0.05,0.2,1,1,0],M.Rw||[10,10],M.QI||[0.1,0.05],ch.nd,ch.a)});}let zb=0,ze=0;const buf=[];const hist=[];
  const nP=Math.round((C.drvDelay+C.drvLag)*60);
  function ref(r){const x=M.x0.slice();if(r){if(r.b!==undefined)x[0]=r.b;if(r.r!==undefined)x[1]=r.r;if(r.v!==undefined)x[2]=r.v;if(r.e!==undefined)x[3]=r.e;if(r.h!==undefined)x[4]=r.h;if(r.d!==undefined&&n>=6)x[5]=r.d;}if(n===7)x[6]=0;return x;}
  let ssC=null;function ssRef(b){if(!ssC||Math.abs(ssC.b-b)>1e-6){const q=L.steady(M,b,0);ssC={b,x:q.x,u:q.u};}return ssC;}
  function step(x,p,r,exc){let xr=ref(r);let cmd,thr;let u0=M.u0;
    if(SS&&M.KA&&r&&r.b!==undefined){const q=ssRef(r.b);xr=q.x.slice();if(r.e!==undefined)xr[3]=r.e;if(n>=7&&n!==11)xr[6]=0;u0=q.u;}
    if(!M.KA){let xp=x.slice(0,n);for(let s=0;s<nP;s++){const u=hist[hist.length-nP+s]||M.u0;xp=xp.map((_,i)=>M.Ad[i].reduce((a,v,j)=>a+v*xp[j],0)+M.Bd[i][0]*u[0]+M.Bd[i][1]*u[1]+M.cd[i]);}
      const dx=xp.map((v,i)=>v-xr[i]);zb=Math.max(-0.5,Math.min(0.5,zb+dx[0]/60));ze=Math.max(-40,Math.min(40,ze+dx[3]/60));const xa=[...dx,zb,ze];
      cmd=M.u0[0]-M.KI[0].reduce((a,kk,j)=>a+kk*xa[j],0);thr=M.u0[1]-M.KI[1].reduce((a,kk,j)=>a+kk*xa[j],0);}
    else{const dx=x.slice(0,n).map((v,i)=>v-xr[i]);zb=Math.max(-ZB,Math.min(ZB,zb+dx[0]/60));ze=Math.max(-ZE,Math.min(ZE,ze+dx[3]/60));
      const z=[...dx,p-u0[0]];for(let j=0;j<ch.nd;j++)z.push((buf[j]!==undefined?buf[j]:p)-u0[0]);z.push(zb,ze);
      cmd=u0[0]-M.KA[0].reduce((a,kk,j)=>a+kk*z[j],0);thr=u0[1]-M.KA[1].reduce((a,kk,j)=>a+kk*z[j],0);}
    if(exc){cmd+=exc[0];thr+=exc[1];}cmd=Math.max(-C.maxSteer,Math.min(C.maxSteer,cmd));thr=Math.max(0,Math.min(1,thr));buf.unshift(cmd);if(buf.length>ch.nd)buf.length=ch.nd;hist.push([cmd,thr]);if(hist.length>60)hist.shift();
    return [cmd,thr];}
  return {step,ch,ss:ssRef,get z(){return [zb,ze];},get zb(){return zb;},get ze(){return ze;}};}
module.exports={mk};
