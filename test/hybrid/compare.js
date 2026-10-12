// Go/no-go test of the hybrid (decision 11 Oct): the physical core's steady slides (hybrid/phys.js) against the slide
// table's measured operating points – each point evaluated at the path radius and slip the identification actually
// held (R + e0, beta0), so only the model is compared, not the regulator's aim.
// node hybrid/compare.js TAB.json [TAB.json …]   env: DRIVE (must match the tables), V=1 (per-wheel detail)
// TAB.json: hybrid/mktab.js's output, or an LSFILE (its gb.dc.* entries are read).
// Per point: speed, road-wheel angle δ, the hands' target p, throttle – table and model – and, with several tables of
// one setup, the table's own scatter (sd over the tables at the same grid point) as the yardstick.
const {loadEngine}=require('../load');const {makeModel}=require('./phys');const fs=require('fs');
const E=loadEngine();const drive=process.env.DRIVE||'awd';E.cfg.drive=drive;E.applyDriveSetup(drive);
const P=makeModel(E,{ruts:!process.env.NORUT}),CAR=E.CAR,f=(x,d=1)=>(x>=0?' ':'')+(+x).toFixed(d),V=process.env.V==='1';
function tables(file){const d=JSON.parse(fs.readFileSync(file,'utf8'));if(d.tab)return [d.tab];
  return Object.entries(d).filter(([k])=>k.startsWith('gb.dc.')&&k.includes('|'+drive+'|')).map(([,v])=>JSON.parse(v));}
const tabs=process.argv.slice(2).flatMap(tables);if(!tabs.length){console.log('no tables for',drive);process.exit(1);}
// the gear: the identification starts in 2 + floor(v0/9) and nobody shifts mid-slide (autoShift)
const gearOf=v=>Math.max(2,Math.min(CAR.gears.length-1,2+Math.floor(v/9)));
const rows=[];
for(const [ti,T] of tabs.entries())for(const m of T.pts){if(!m.ok)continue;
  const x0=m.x0,u0=m.u0,rho=m.R+x0[3],beta=x0[0],v0=x0[2],gi=gearOf(v0);
  // start: the table's point; wheels at their contact patch's speed + 10 %
  const vx=v0*Math.cos(beta),vy=v0*Math.sin(beta),r=v0/rho,w=E.WHEELS.map(Wh=>1.1*Math.hypot(vx-r*Wh.ry,vy+r*Wh.rx)/CAR.Rw);
  const g=[v0,x0[5],u0[1]];
  const q=P.equilibrium(rho,m.R,beta,gi,g,true);
  rows.push({ti,R:m.R,b:Math.round(m.b*57.3),beta,rho,gi,tab:{v:v0,d:x0[5],p:u0[0],thr:u0[1],r:x0[1]},mod:q,ok:q.fn<1e-6});}
// the table's own scatter: the same grid point over the tables
const key=r=>r.R+'/'+r.b,grp={};for(const r of rows)(grp[key(r)]=grp[key(r)]||[]).push(r);
const sd=(a)=>{const m=a.reduce((s,x)=>s+x,0)/a.length;return Math.sqrt(a.reduce((s,x)=>s+(x-m)**2,0)/Math.max(1,a.length-1));};
console.log(`${drive}, ${tabs.length} table(s), ${rows.length} held points | per point: table → model (model − table)`);
console.log('  R/b    tab β  ρ      | v km/h               | δ °                  | p °                  | throttle            | gear rpm  conv');
const err={v:[],d:[],p:[],thr:[]};
for(const r of rows.sort((a,b)=>a.R-b.R||a.b-b.b||a.ti-b.ti)){const t=r.tab,m=r.mod;
  const dv=(m.v-t.v)*3.6,dd=(m.delta-t.d)*57.3,dp=(m.p-t.p)*57.3,dth=m.thr-t.thr;
  if(r.ok){err.v.push(dv/(t.v*3.6));err.d.push(dd);err.p.push(dp);err.thr.push(dth);}
  console.log(`  ${(r.R+'/'+r.b).padEnd(6)} t${r.ti} ${f(r.beta*57.3,1)} ${f(r.rho,1)} |${f(t.v*3.6,1)} →${f(m.v*3.6,1)} (${f(dv,1)}) |${f(t.d*57.3,1)} →${f(m.delta*57.3,1)} (${f(dd,1)}) |${f(t.p*57.3,1)} →${f(m.p*57.3,1)} (${f(dp,1)}) |${f(t.thr,2)} →${f(m.thr,2)} (${f(dth,2)}) | ${m.rpm?CAR.gears[r.gi].n:''} ${Math.round(m.rpm)} ${r.ok?'ok':'NO '+m.fn.toExponential(1)}${m.T.lift?' lift':''}`);
  if(V&&r.ok)console.log('      wheels κ',m.T.per.map(p=>f(p.kappa,2)).join(' '),'| α°',m.T.per.map(p=>f(p.alpha*57.3,1)).join(' '),'| s',m.T.per.map(p=>f(p.s,2)).join(' '),'| Fz',m.T.Fz.map(x=>Math.round(x)).join(' '),'| Td',m.Td.map(x=>Math.round(x)).join(' '));}
if(tabs.length>1){console.log('table scatter (sd over tables at one grid point; slip differs between them):');
  for(const [k,a] of Object.entries(grp)){if(a.length<2)continue;console.log(`  ${k.padEnd(7)} n${a.length} β ${f(sd(a.map(r=>r.beta*57.3)),1)}° | v ${f(sd(a.map(r=>r.tab.v*3.6)),1)} km/h | δ ${f(sd(a.map(r=>r.tab.d*57.3)),1)}° | thr ${f(sd(a.map(r=>r.tab.thr)),2)}`);}}
const st=a=>a.length?`mean ${f(a.reduce((s,x)=>s+x,0)/a.length,2)} rms ${f(Math.sqrt(a.reduce((s,x)=>s+x*x,0)/a.length),2)} max|.| ${f(Math.max(...a.map(Math.abs)),2)}`:'–';
console.log(`SUMMARY ${drive}: converged ${err.v.length}/${rows.length} | speed (rel.) ${st(err.v.map(x=>100*x))} % | δ ${st(err.d)}° | p ${st(err.p)}° | throttle ${st(err.thr)}`);
