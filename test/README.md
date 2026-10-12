# Test harness

The physics of `../index.html` runs here without a browser: the script block is extracted, loaded into a Node `vm` context with DOM stubs, and driven through standard manoeuvres. Every change to the simulation can be compared with the state before it in a couple of minutes.

## Requirements

- Node 18 or newer, Python 3
- optional: `npm install` in the repository root (`@napi-rs/canvas` for canvas renders, `node-web-audio-api` for sound demos, `playwright` for browser screenshots – then `npx playwright install chromium`)

## Usage

```
python3 extract.py            # writes engine.js from ../index.html (or pass a path)
MATRIX_PART=1 node consistency.js   # scenario matrix part 1 (≈ 80 scenarios), 2 (≈ 45: phantom, tutor, drift), 3 (race autopilot); each under 5 min, none = all
node bench.js                 # reference figures: 0–100, braking, skidpad, step steer, wheel dynamics
```

`consistency.js` prints only findings (ERROR / WARN / INFO) plus key figures. It covers, for all-wheel, front-wheel and rear-wheel drive on tarmac, gravel and snow: rest state, acceleration, braking with and without ABS, skidpad left and right, step steer, load change (lift-off and throttle stab), catching a slide, handbrake, straight-line stability, top speed, determinism, phantom laps with elevation profile and road state, gear changes, physical invariants (ΣFz = m·g, ay = v·r, forces inside the friction circle, deceleration ≤ µ·g, acceleration ≤ P/(m·v)), reverse, reset, state separation between player and phantom, and braking with steering for raw and threshold keyboard braking.

The CI workflow fails on `ERROR` findings; `WARN` and `INFO` are reported.

## Scenario scripts

| Script | Topic |
|---|---|
| `susptest.js` | suspension: landing, heave decay, rough gravel, brake dive |
| `rwdtest.js` | rear-wheel drive: recovery from oversteer |
| `hbtest.js` | handbrake turns |
| `difftest.js` | differentials: wind-up, throttle stab, lift-off, shift shock |
| `roadtest.js` | elevation profile: phantom lap flat vs. profile |
| `kintest.js` | steering kinematics: toe, roll steer |
| `straight.js` | straight-line stability |
| `braketurn.js`, `brakerel.js`, `brakesteer.js` | braking with steering, brake release, threshold braking |
| `lifttest.js` | lift-off in a steady corner |
| `review.js` | review items M1, M4–M6: braking on a 15 % grade, sand against gravel, wet grip over speed, puddles on loose ground |
| `apc.js`, `ph.js` | autopilot laps (catch, yaw kicks, drift) and a single phantom lap |
| `acceptance.sh` | acceptance suite for autopilot changes: 11 cases (random stages 19029, 9968 and 4711, circuit and stage, AWD/RWD/FWD, grip and slides), driven as the browser drives; bar: all on the road |
| `acc-robust.sh` | the acceptance suite's slide cases driven again from a start moved k × 5 cm to the side (`drstat.js JIT=k`), two settings side by side (`A`, `B` as SETA JSON, `JITS`, `CASES`): per case runs off the road and the closest approach to the edge – whether a case on the road is robust or lucky (12 Oct: robust to the start; the close spots are systematic) |
| `drift-suite.js` | eight autopilot scenarios at a limit-range setting (`node drift-suite.js 1`, or `0` for grip), two at a time; bars: on the road, at most 3 visible steering corrections per second; reports slide shares and lengths |
| `drift-bench.js` | one autopilot run with slide metrics (sideways share on the planned stretches, slides and their length, corrections per second); `OVAL=R,straight` drives a stadium oval; `REACT` sets the driver's reaction, `SETA`/`SETC`/`SETS` override AP/CAR/gravel fields for trials – `SETA='{"dcUse":true}'` switches the drift controller on for every drive, `false` off (default since 12a: on for RWD only; it measures its table first, ~10 s); the off-road time names where it began (`off 0.3 (from s 1501)`); `REVLOG=1` counts the visible steering corrections by who steered (`2`: each one) |
| `dc-entry.js` | the drift controller's corners one by one: engaged, built, held, given up, off; within ±4° and the slip error (mean ± sd) on the held part; `OVALS="40,150 83,250"` (straight 0: a ring), `TRACK`, `SEED`, `NL`, `DRIVE`, `SETA`, `QI`, `AMP` (road roughness), `NODRIVEN`, `V=1` a line per corner |
| `same.js` | bit-identity of two engine builds (`node same.js engine_10b.js engine.js`): 15 scripted drives over drives, surfaces, rain, hazards, ABS/TC, the whole state compared every second – for refactors of `step()` |
| `hybrid/` | the hybrid drift model (11 Oct): the go/no-go test's model (`phys.js`) against the slide tables (`compare.js`) and the engine's own holds and steps (`engstep.js` + `eng.py`, `summary.py`); the reductions to the controller's model (`ctlmodel.py`, `modal.py`, `eigfit.py`, `reduce.py`); the game's slide model checked (`incheck.js`, `eigph.js`, `lincheck.js`), its validation diagnosed (`valdiag.js`), a corner traced (`dctrace.js`), tables measured/computed (`mktab.js`), the default autopilot against 10b (`apsame.js`) – see `hybrid/README.md` |
| `dc/` | the drift controller's ring bench – identification, validation, chains of operating points; `dc/bench.sh` / `dc/bench3.sh` (the corner set over one / three slide tables), `dc/stagebench.sh` (the controller on five stages), `dc/chkbench.sh` (the corner check over ~50 corners, RWD and AWD), `dc/robust.js` + `dc/robust.py` (repeated identifications, every gain on every model) – see `dc/README.md` |
| `drstat.js` | one autopilot run as the browser drives it (`SEED`, `DRIVE`, `SURF`, `SETUP`, `SETX`, `SETA`; `EP=1` prints each excursion with the seconds before; `JIT=k` starts k × 5 cm to the side); reports the closest approach of the car's centre to the road's edge (`edge 0.3 m at s 1223`) |
| `laps.js` | race autopilot over several laps: lap time, off track, catch |
| `env.js` | race autopilot (measured envelope + speed profile + catch) or the old envelope driver, at a pace, optionally with yaw kicks |
| `pace.js` | phantom or autopilot with the plan raised towards the limit (`aLat`, `aBrk`), optionally with drift: lap, off track, spinning, held angle |
| `shot.js` | headless renders of the driver or top view, canvas only |
| `uishot.js` | browser screenshots with the full interface, for the README |
| `soundtest.js` | sound demos rendered to WAV |

All scripts accept `ENGINE=other-engine.js node …` to run against an older engine extracted from an older HTML (rename the file after extracting it).

Storage: `LSFILE=file.json` keeps the engines' localStorage (envelopes, slide tables, racing lines) in a file across runs – for trials that change only the driving; delete it when the planning or measurement changes. `apPrepare()` computes the whole plan at once (envelope, slide table, line).

## Writing a scenario

`loadEngine()` returns the simulation API: `S` (state), `IN` (inputs), `cfg` (setup), `CAR`, `VERT`, `SURF`, `TRACK`, `LINE`, `step(dt)`, `readInput(dt)`, `selectTrack(id)`, `ghostAI()`, and more (see `extract.py`). A scenario sets the configuration, places the car with a speed, applies inputs per step and reads the state. Ten lines are usually enough; see `lifttest.js` for a compact example.
