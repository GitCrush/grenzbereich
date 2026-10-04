# Test harness

The physics of `../index.html` runs here without a browser: the script block is extracted, loaded into a Node `vm` context with DOM stubs, and driven through standard manoeuvres. Every change to the simulation can be compared with the state before it in a couple of minutes.

## Requirements

- Node 18 or newer, Python 3
- optional: `npm install` in the repository root (`@napi-rs/canvas` for canvas renders, `node-web-audio-api` for sound demos, `playwright` for browser screenshots – then `npx playwright install chromium`)

## Usage

```
python3 extract.py            # writes engine.js from ../index.html (or pass a path)
node consistency.js           # scenario matrix, ≈ 117 scenarios, 2–3 min
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
| `shot.js` | headless renders of the driver or top view, canvas only |
| `uishot.js` | browser screenshots with the full interface, for the README |
| `soundtest.js` | sound demos rendered to WAV |

All scripts accept `ENGINE=other-engine.js node …` to run against an older engine extracted from an older HTML (rename the file after extracting it).

## Writing a scenario

`loadEngine()` returns the simulation API: `S` (state), `IN` (inputs), `cfg` (setup), `CAR`, `VERT`, `SURF`, `TRACK`, `LINE`, `step(dt)`, `readInput(dt)`, `selectTrack(id)`, `ghostAI()`, and more (see `extract.py`). A scenario sets the configuration, places the car with a speed, applies inputs per step and reads the state. Ten lines are usually enough; see `lifttest.js` for a compact example.
