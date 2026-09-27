# TODO

Open work, roughly in order of priority. Every physics change must keep `test/consistency.js` clean and should add a scenario that pins the new behaviour. Run `npm test` before and after; compare `npm run bench` figures.

## 1. Done: axle differential locking value (see CHANGELOG)

The axle LSD is now a Coulomb constraint with the conventional locking definition; defaults re-measured (`test/lsdsweep.js`, `test/tpltest.js`). Setup templates exist (`SETUP_TEMPLATES`); item 4 below is reduced to adding a Snow template if wanted.

## 2. Dynamic road state (phase D)

Each pass should sweep the line and deepen the ruts, instead of the static band and ruts around the ideal line (`ROAD`, `lineRel`, `gripFactor`, `rutZ`, `rutSlope`).
- Grid along the track (s × n, e.g. 1 m × 0.25 m) holding loose-material depth and rut depth.
- Each wheel pass moves loose material outwards proportional to lateral sliding and wheel load; rolling compacts; ruts deepen with passes at the same n.
- Calibration reference: DiRT Rally 2.0's degradation — swept after ≈5 passes, best state around 10, rutted with a berm from ≈15.
- Setup: "road state: fresh / driven line / dynamic". Reset with the track.
- Rendering: band and ruts from the grid in both views.

## 3. Phantom driver for two-wheel drive

The phantom (`ghostAI`) is tuned for all-wheel drive. As FWD and RWD on gravel it runs off track for 4–5 s per lap (matrix findings). Needs drive-specific throttle metering on corner exit, earlier lift for FWD understeer, counter-steer gain and throttle recovery for RWD oversteer.

## 4. Setup presets

Gravel, Tarmac and RWD drift exist as templates. Open: a Snow template (softer, more preload, 70 % brake bias), and measuring every template in the matrix rather than only in `test/tpltest.js`.

## 5. Two-wheel drive on snow with held full throttle

0–100 km/h in 16–17 s with the throttle held flat (matrix WARN). The wheels spin at the limiter; with metered throttle the car is normal. Decide whether this is an input-model question (keyboard throttle metering like the threshold brake) or acceptable; if acceptable, turn the WARN into an INFO in the matrix.

## 6. Tyre temperature and wear

Not modelled. Minimum version: one thermal node per tyre (surface/carcass lumped), heating from sliding power, cooling with speed, grip factor over a temperature window; wear as a slow loss of peak µ. Keep off by default until measured.

## 7. Module split with a build step

The single file (≈3500 lines) is good for distribution and bad for collaboration. Split into physics / tyre / suspension / drivetrain / input / rendering / sound / UI as ES modules, with a small build script that still produces the single `index.html`. `test/extract.py` then loads the modules directly instead of cutting the script block.

## 8. Analogue clutch axis

Map a clutch pedal axis (gamepad/wheel) and bypass the launch and stall-protection clutch model when the axis is present; the pedal travel then sets the clutch capacity directly. Needs hardware to test.

## 9. Mobile controls: test on devices

The swipe/thumb-wheel controls were built and rendered headless only. Check on a phone: zone widths (42 % / 52 % split), wheel radius, ratio 2.2, whether pointer capture works on iOS Safari, and whether the handbrake button collides with the pedal zone.

## Minor

- Reverse gear: 57 km/h after 4 s at half throttle; check the ratio.
- `test/consistency.js` messages and the test README could list the setup used per scenario.
