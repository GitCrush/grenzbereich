// The family of steady slides at one radius, from grip to the slide (decision 11 Oct: can the controller move along it
// into the slide?) – continuation from 22° down and up; per slip: speed, steering, hands' target, throttle, wheels' slip
// speed, gear. node hybrid/family.js R   env: DRIVE
const {loadEngine}=require('../load');const E=loadEngine();const drive=process.env.DRIVE||'awd';E.cfg.drive=drive;E.applyDriveSetup(drive);E.selectTrack('rundkurs');
const P=E.phSetup(),R=+process.argv[2],f=(x,d=1)=>(x>=0?' ':'')+(+x).toFixed(d);E.AP.tNoBlend=true;
const M22=E.phPoint(P,R,22/57.3,[0.92*Math.sqrt(P.sur.mu*9.81*R),drive==='rwd'?-0.29:0.06,0.4]);
const row=(b,M)=>{const q=M.q;console.log(`${drive} R${R} β${f(b,0)}: ${M.ok?'ok ':q&&q.fn<1e-6?'sat':'-- '} v ${f(q.v*3.6)} δ ${f(q.delta*57.3)} p ${f(q.p*57.3)} thr ${f(q.thr,2)} slip ${M.x0&&M.x0[6]!==undefined?f(M.x0[6],2):'–'} m/s gear ${q.gi} fn ${q.fn.toExponential(0)}`);};
row(22,M22);let last=M22;
for(let b=20;b>=2;b-=2){const M=E.phPoint(P,R,b/57.3,[last.q.v,last.q.delta,last.q.thr]);row(b,M);if(M.q.fn<1e-6)last=M;}
