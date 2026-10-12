// Part 2 of the go/no-go test (decision 11 Oct): the physical core's local dynamics against the identified models.
// For each held point of the table(s): the steady slide at the point's own (R + e0, beta0), the linearisation of the
// dynamics there (hybrid/phys.js linearise: A, B, C), written with the identified model (Ad, Bd, x0, u0, r2) to a JSON
// file for hybrid/dyn.py, which compares step responses and the slow poles.
// node hybrid/dyn.js OUT.json TAB.json [TAB.json …]   env: DRIVE
const {loadEngine}=require('../load');const {makeModel}=require('./phys');const fs=require('fs');
const E=loadEngine();const drive=process.env.DRIVE||'awd';E.cfg.drive=drive;E.applyDriveSetup(drive);
const P=makeModel(E),CAR=E.CAR;
function tables(file){const d=JSON.parse(fs.readFileSync(file,'utf8'));if(d.tab)return [d.tab];
  return Object.entries(d).filter(([k])=>k.startsWith('gb.dc.')&&k.includes('|'+drive+'|')).map(([,v])=>JSON.parse(v));}
const tabs=process.argv.slice(3).flatMap(tables),gearOf=v=>Math.max(2,Math.min(CAR.gears.length-1,2+Math.floor(v/9)));const out=[];
for(const [ti,T] of tabs.entries())for(const m of T.pts){if(!m.ok)continue;const x0=m.x0,u0=m.u0,rho=m.R+x0[3],beta=x0[0],v0=x0[2];
  const vx=v0*Math.cos(beta),vy=v0*Math.sin(beta),r=v0/rho,w=E.WHEELS.map(Wh=>1.1*Math.hypot(vx-r*Wh.ry,vy+r*Wh.rx)/CAR.Rw);
  const g=[v0,x0[5],u0[1]];const q=P.equilibrium(rho,m.R,beta,gearOf(v0),g,true);if(q.fn>1e-6)continue;
  const L=P.linearise(q);
  out.push({ti,R:m.R,b:Math.round(m.b*57.3),blend:!!m.blend,r2:m.r2,Ad:m.Ad,Bd:m.Bd,x0,u0,A:L.A,B:L.B,C:L.C,f0:L.f0,eq:{v:q.v,delta:q.delta,thr:q.thr,p:q.p,beta,rho}});}
fs.writeFileSync(process.argv[2],JSON.stringify({drive,pts:out}));console.log(drive,'points',out.length,'max |f0|',Math.max(...out.map(o=>Math.max(...o.f0.map(Math.abs)))).toExponential(2));
