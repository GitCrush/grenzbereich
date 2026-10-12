# The hybrid drift model – go/no-go test (11 Oct 2026)

A hybrid that integrates the existing drift models, proposed 10 Oct and put to a go/no-go test on 11 Oct. The question: can a physical model whose parameters come from the engine
replace or carry the identified slide models (`dcTableGen`), whose weak points are their scatter between
identifications, their gaps, and that nothing is known outside their operating points?

## The model – `phys.js`

`makeModel(E, {ruts})` builds the engine's car at steady state, every number read from the loaded engine (`CAR`, `cfg`,
the surface, `VERT`, `ROAD`, `WHEELS`, `RUT_HALF`/`RUT_W`). Kept formula by formula from `step()`: the tyre law per wheel,
the gravel plough, Ackermann, toe and bump steer, steering compliance, rolling resistance, aero, the engine map with
antilag boost, axle diffs and the rigid centre, the hands' compliance in a slide, the band's grip and loose share, the
ruts' height (loads) and wall force. Reduced: loads from the steady solution of the suspension's equations, no tyre
relaxation, no steering or wheel dynamics in the steady state.

- `equilibrium(rho, R, beta, gear, [v, delta, thr], near)` – the steady slide on a circle of radius `rho` at slip
  `beta` (Newton, numerical Jacobian, continuation over the diffs' plate slip, local steps). Several slides can exist
  at one slip (AWD near 15°); `near` returns the one closest to the guess. About 7 ms with a neighbour's solution as
  the start, 13 ms cold.
- `deriv`, `linearise(q)` – the dynamics (body, four wheels with the engine's inertia, turbo boost) and their
  linearisation at a steady slide (1.4 ms).

## The tests

| script | what |
|---|---|
| `mktab.js OUT.json` | measure a slide table (env `DRIVE`, `SEEDOFF`, `SETA` e.g. `{"tNoBlend":true}`, `LSFILE`) |
| `compare.js TAB…` | the model's steady slides against the table's operating points (at the slip and radius each point held); env `NORUT=1` the model without ruts |
| `dyn.js`, `dyn.py` | the model's step responses against the identified models' (no judge – superseded by `engstep`) |
| `engstep.js OUT.json TAB…` | **the engine as the judge**: per table point a clean hold (the table's regulator, no excitation, 5 s) and open-loop steps from it (steering ± 1°, throttle ± 0.05, 0.5 s, (y+ − y−)/2); env `NORUT=1` ruts off in engine and model |
| `eng.py ENG.json` | fit of the identified model's and the physical model's predicted responses to the engine's, driven by the engine's recorded inputs (1 = exact) |
| `summary.py ENG…` | the model's steady slides against the engine's steady holds |

## Results (gravel, reaction 70 %; AWD three tables, RWD two)

**Steady slides against the engine's clean holds** (holds steady in slip and steering; model at the hold's radius and slip):

| | holds | steering | throttle | speed |
|---|---|---|---|---|
| RWD, ruts | 23 | 0.08° rms (max 0.21°) | 0.03 rms | 0.7 % rms |
| AWD, ruts | 13 | 0.39° rms (max 1.1°) | 0.01 rms | 0.7 % rms |
| AWD, no ruts | 34 | median 0.21° (two outliers: one at full throttle – no steady slide exists –, one solved on another branch of the family) | median 0.00 | median 0.4 % |

**Step responses** (median fit over the steady holds, identified → physical):

| | steer → β | steer → r | steer → v | throttle → β | throttle → r | throttle → v |
|---|---|---|---|---|---|---|
| AWD (no ruts, 48) | 0.09 → 0.81 | −0.09 → 0.89 | 0.45 → 0.99 | 0.58 → 0.90 | 0.38 → 0.71 | 0.66 → 0.96 |
| AWD (ruts, 41) | 0.18 → 0.79 | −0.16 → 0.86 | 0.59 → 0.97 | 0.53 → 0.86 | 0.26 → 0.63 | 0.63 → 0.91 |
| RWD (ruts, 23) | 0.62 → 0.85 | 0.72 → 0.93 | 0.10 → 0.92 | 0.73 → 0.86 | 0.66 → 0.86 | 0.64 → 0.95 |
| AWD band models alone (ruts, 15) | 0.45 → 0.81 | 0.50 → 0.89 | 0.66 → 0.98 | 0.14 → 0.89 | −0.59 → 0.75 | 0.61 → 0.93 |

Findings:
- **Go.** The physical model reproduces the engine's steady slides to a fraction of a degree of steering and its
  local dynamics far better than the identified models.
- The identified models overestimate the yaw response to steering 1.5–5 × (AWD) and put the slide's unstable pole at
  +1 … +6 /s where the engine's is near +0.1 … +1.3 /s – the closed-loop identification with hidden wheel speeds is
  biased; averaging band and loose models is not the cause (the band's alone are not better overall).
- The table's operating points were measured under excitation and are not steady slides: the clean hold at the same
  grid point settles elsewhere (AWD R 75 m: table 22° / +3.7° steering, clean hold 26–28° / 0 to −3°).
- The ruts matter: without them in the model its steering was 1–3° off the engine's AWD holds; with ruts in the
  engine only 13 of 54 AWD holds were steady (35 without) – the ruts under the wheels swing the slide.
- The ruts are a property of the place, not of the car: their force and height under each wheel change over the rut's
  width (0.32 m). Against the table's points (driven under excitation, i.e. averaged over positions) the model at one
  fixed position is 1.7° rms off in steering without ruts and 3.8° rms with them – off by 4–11° at AWD 16°, where the engine
  itself holds no steady slide with ruts. In the game they must enter as a term of the lateral position relative to
  `LINE` (or averaged over the expected spread), and the controller must treat them as a disturbance.
- Still to improve in the physical model: throttle → yaw rate (0.63–0.75; the load transfer's lag through the body's
  roll and pitch, tyre relaxation), the solver's branch choice (continuation along the slip).

## In the game (build 2026-10-11a)

The slide model is ported to `index.html` ("THE SLIDE MODEL", before `dcTableGen`) on the engine's own tyre and engine
functions (factored out of `step()` bit-identically – `test/same.js`); `incheck.js` checks that its steady slides satisfy
this folder's model to 10⁻⁹. The controller's model: what was learned on the way (scripts in this folder):

| reduction | slow modes | fit to the engine's steps (steer / throttle → β r v) |
|---|---|---|
| six states, rows fitted over 0.5 s (`ctlmodel.py`) | unstable pair +2 ± 1.2j /s (spurious) | 0.85–0.94 / 0.73–0.91 |
| six states, fitted over 6 s (`eigfit.py`) | right | RWD 0.41–0.79 |
| three exact modes (`modal.py`, modal3) | exact | below zero (throttle) |
| **eight states (+ wheel slip speed, boost), fitted over 2 s** (`modal.py`, fit5) | right (RWD: −0.09 −2.41 +3.39 −6.24 −10 vs −0.10 −2.42 +3.39 −6.28 −10) | 0.91–0.96 / 0.73–0.97 |

`lincheck.js`: the regulator designed on the eight-state model is stable on the full linearised core. `valdiag.js`
repeats a point's validation with the road rough/smooth and the ruts on/off; `dctrace.js` traces the controller through
a corner (slip, reference, steering, throttle, speed against the plan, the wheels' slip, the gear asked for).

## The bar measured (12 Oct, night)

| script | what |
|---|---|
| `spot.js S0 S1` | one stretch traced every 0.1 s: slip, target, speed and plan, offset from the centre and the road's half-width, the racing line's offset, yaw rate, steering, pedals, `DC` while the controller drives |
| `dctrace.js` | also `TRACK` (a named track) and `SR="s0,s1"` (every 0.1 s on that stretch, controller on or not) |
| `corners.js` | the controller's candidate corners on a track, the corner check's verdict on each, the corners kept; `SEG=s` traces the check's drive of one corner (`AP.tChkLog`) |

