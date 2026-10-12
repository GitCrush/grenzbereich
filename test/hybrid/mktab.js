// Measure (or load from LSFILE) the slide table of one setup and write it as JSON (decision 11 Oct: the hybrid model's
// go/no-go test compares its equilibria with the table's operating points).
// node hybrid/mktab.js OUT.json   env: DRIVE (awd|rwd|fwd), SURF (default gravel), SEEDOFF (AP.dcSeedOff), SETA (JSON for AP, e.g. {"tNoBlend":true}: the band's models alone), LSFILE
const {loadEngine}=require('../load');const E=loadEngine();
if(process.env.SURF)E.cfg.surface=process.env.SURF;if(process.env.DRIVE){E.cfg.drive=process.env.DRIVE;E.applyDriveSetup(E.cfg.drive);}
E.AP.dcSeedOff=+(process.env.SEEDOFF||0);if(process.env.SETA)Object.assign(E.AP,JSON.parse(process.env.SETA));E.AP.drift=1;E.AP.dcUse=true;E.selectTrack('rundkurs');E.AP.on=true;E.apPrepare();
const T=E.AP.dcTab;require('fs').writeFileSync(process.argv[2],JSON.stringify({drive:E.cfg.drive,surface:E.cfg.surface,seedOff:E.AP.dcSeedOff,cfg:E.cfg,tab:T}));
console.log('table',E.cfg.drive,E.cfg.surface,'seedOff',E.AP.dcSeedOff,'points',T.pts.length,'ok',T.pts.filter(m=>m.ok).length);
