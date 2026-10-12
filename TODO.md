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

Since the speed plan shares the tyre between braking and cornering (CHANGELOG), all twelve phantom laps (circuit and stage, tarmac and gravel, AWD/RWD/FWD) are on the road. Still open: drive-specific throttle metering on corner exit and counter-steer for RWD oversteer, measured at a raised plan (`test/pace.js`); for RWD a way to steady a slide under braking without letting off the front-biased brake (letting off leaves the engine braking the rear alone – tried, spins).

## 4. Setup presets

Gravel, Tarmac and RWD drift exist as templates. Open: a Snow template (softer, more preload, 70 % brake bias), and measuring every template in the matrix rather than only in `test/tpltest.js`.

## 5. Done: two-wheel drive on snow (see CHANGELOG)

Traction-bound, not an input question: an ideal slip controller needs 14.7 s (RWD) / 15.6 s (FWD) to 100 km/h, the keyboard metering 15.7 / 16.3 s. Open only if the snow µ itself is questioned (0.52 for every tyre; studded rally tyres grip far more – review G4).

## 6. Tyre temperature and wear

Not modelled. Minimum version: one thermal node per tyre (surface/carcass lumped), heating from sliding power, cooling with speed, grip factor over a temperature window; wear as a slow loss of peak µ. Keep off by default until measured.

## 7. Module split with a build step

The single file (≈3500 lines) is good for distribution and bad for collaboration. Split into physics / tyre / suspension / drivetrain / input / rendering / sound / UI as ES modules, with a small build script that still produces the single `index.html`. `test/extract.py` then loads the modules directly instead of cutting the script block.

## 8. Analogue clutch axis

Map a clutch pedal axis (gamepad/wheel) and bypass the launch and stall-protection clutch model when the axis is present; the pedal travel then sets the clutch capacity directly. Needs hardware to test.

## 9. Mobile controls: test on devices

First device test done (thumb crank impractical → slider added and made the default; pedal travel capped by the available room; overlays scroll). Still to check on a phone: slider track length and knob size, whether pointer capture survives the thumb leaving the canvas on iOS Safari, handbrake button versus the pedal zone, portrait layout.

## 10. Snowbanks as physics

The snowbanks on snow are visual only. On Swedish stages drivers lean on them in corners; as a physical element they would be a soft wall beside the road: lateral force growing with penetration, high damping, some drag, maybe losing height where hit (links to the dynamic road state, item 2). Until then the visual bank promises a support the car does not get.

## 11. Open points from the review of the vehicle model

Applied: K1, K2, M1, M2, M3, M4, M5, M6, M7 (HUD part), G8. Estimates that want a source: the wet-tarmac fall with speed for rally rain tyres (M5 uses Bosch's car-tyre table), the mud grip in a puddle on loose ground (M6, ×0.70). Open: G1 (standstill without static friction, camber forces at rest), G2 (sand rolling resistance only when sliding), G3 (2.5 m point spacing assumed in hazards and pace notes), G4 (ice and snow-patch grip independent of the tyre), G5 (side aero area), G6 (engine drag torque), G7 (heave with total instead of sprung mass, roughness takes no energy). The lateral term −ay·cross slope in the apparent gravity carries a spurious −g·slope² (under 1 % at 10 % cross slope).

## Minor

- Reverse gear: 57 km/h after 4 s at half throttle; check the ratio.
- `test/consistency.js` messages and the test README could list the setup used per scenario.


## Autopilot – real drifting (goal, 7 Oct: 0 % grip at the limit, 100 % controlled use of the limit range, both at maximum speed; at 100 %, long drawn-out drifts)
State (build 2026-10-09l, see CHANGELOG): physical steering for everyone, a human driver model for the drivers in the code (*Autopilot reaction*, default 70 %), the line as a corridor. Acceptance 11 of 11 on the road; `test/drift-suite.js` 8 of 8 at 0 % and at 100 % (reaction 70 %). **Open: long slides on circuits and stages** – 0.5–1.6 s on average at 10–15°; on the oval up to 7 s.
What is known (measured, see CHANGELOG 09i – l):
- The slides are short because the hold law cannot hold an angle: run on the measuring rings at a target angle it held 1–3 of 16 points within ±4°, with the human driver and with the machine alike. The drift equilibrium on gravel is not steady under constant throttle (on R 83 m the angle wanders 5° → 40° over 5–8 s), and lifting turns an AWD car further in before, 1–2 s later, less throttle means less angle.
- A hold law steering on the anticipated slip angle (thF + 0.3·(path yaw − r) + 3·(β − target), anticipated over the reaction time, throttle with a slow trim) holds 20° ± 2° on a R 83 m ring – but in the race the car then runs 4–9 m wide; angle and path need both inputs. The speed at which the angle is held is the path's lever: 5 % too slow and the slide dies under the plan's brake, too fast and it runs wide (the safety rules now plan drift corners 5 % under the measured equilibrium – that is one reason the slides are short).
Regressions to fix (build 09l against 09h): race autopilot, tarmac stage RWD, 100 %: 0.6 s off (the off-line speed cut brakes mid-corner – found 10 Oct: the slide timer had been dead since 09k, see CHANGELOG 10a); slides at 60 % RWD on snow (matrix part 2): 1.1–1.7 s off and 5–8 s slower; `drift-suite.js 1`: the oval at 3.06 corrections per second (bar 3).
State 10 Oct (build 2026-10-10a): **the drift controller exists** (identified slide models + LQI that knows the driver's delay, see CHANGELOG and the block before `driftEqAt`) and holds slides on the ring bench (AWD/RWD, R 22–150 m, 15–28°, reaction 0–100 %, rough road: β ±0.5–3.5°, path ±0.2–1.6 m) and on the oval (6–10 s at 20–25°, on the road). It is **off in the game** (`AP.dcUse`) because on the circuit and the stages it left the road for 2–60 s per lap.
State 10 Oct evening (build 2026-10-10b): the controller is robust and safe where it is used – geometric path rows, weights searched for the worst case over repeated identifications, band and loose-gravel models averaged, each point validated with the game's regulator, each corner driven in the planning world before it is planned (see CHANGELOG 10b). Corner benchmark (`test/dc/bench3.sh`, three tables): 37 slides, none given up, none off the road. **Still off by default.**
**Plan for 11 Oct – a hybrid model, integrating the existing drift models.** Why: the table's weaknesses are its scatter between identifications (18–24° at the same setup; a gain from one run unstable on another's model), its gaps (points that never settle), and that nothing is known outside its operating points (the way into the slide – the RWD start –, corners that tighten). The most effective single step of 10b was replacing identified rows by physics (`dcKin`, the path rows). The hybrid takes that further:
- *Physical core:* a reduced nonlinear model (single track with load transfer, the engine's tyre law per axle, the drive split, the driver's chain), its parameters read from the engine's own car and surface definitions – no second set of numbers to maintain.
- *The existing models as data, three roles:* (1) **calibration** – the table's operating points (speed, steering, throttle, yaw rate at each R/β, band and loose gravel, with the scatter of repeated identifications) fit a handful of correction factors (effective grip per axle, driven-axle force, steering offset) by least squares weighted by the inverse scatter; (2) **row-wise fusion** – per state row the physics Jacobian is the prior and the identified row the measurement, weighted by the row's fit (R²) and its scatter between runs (`dcKin` is the limit case: geometry exact, weight 1); the steering chain and the throttle response stay identified where the physics is weak; (3) **residuals** – what is left (Δu0, ΔA over R/β) as smooth fields, fading to pure physics outside the grid.
- *What the physics adds:* operating points and linearisations anywhere (no gaps, models along tightening corners), a feedforward path into the slide from the inverse model (the breakaway for RWD), gains scheduled along the corner.
- *The engine stays the judge:* the ring validation and the corner check are unchanged.
- *First step, a go/no-go test:* the physics model's drift equilibria against the table's points (AWD and RWD, gravel): speed, steering, throttle, yaw rate; then its Jacobians against the identified rows with a good fit. Small errors → build it; large → the errors show which effect of the engine the model lacks.
- **Done 11 Oct – go** (tools and numbers in `test/hybrid/README.md`). The core (`test/hybrid/phys.js`: the engine's tyre law, steady loads from its suspension, diffs and centre, ruts – all parameters read from the engine) against the engine's own clean holds and step responses: steady slides RWD 0.08° steering / 0.03 throttle / 0.7 % speed rms, AWD median 0.2° / 0.01 / 0.4 %; step responses fit 0.8–0.99 where the identified models fit −0.2–0.7 (they overestimate steering → yaw 1.5–5 × and the slide's instability). The table's operating points, measured under excitation, are not steady slides. The ruts are the open point: their effect depends on the lateral position at the rut's width (AWD 16° with ruts: the engine holds no steady slide, the model at one position is 4–11° off the table's position-averaged points) – in the game a term of the offset from `LINE` and a disturbance for the controller. 7 ms per steady slide, 1.4 ms per linearisation – a grid in about a second, no measuring drives.
- *What that changes in the plan:* the physics carries operating points and the vehicle rows; the identification's role shrinks to what the physics lacks – the yaw row's throttle response, the steering chain – and to a check. Next steps: (1) the core into the game – the tyre law factored out of `step()` into one function both use (bit-identical, checked by the regression tests), the steady solver with continuation along the slip; (2) `dcTable` from the core (grid by computation, the driver chain's rows as now, ruts along `LINE`), the gains on its linearisation, validation and corner check unchanged; (3) the bench and suites against build 10b; (4) the way into the slide (RWD breakaway) from the core's dynamics.
- **Done 11 Oct (build 2026-10-11a, see CHANGELOG):** (1) and (2) – the shared laws bit-identical, the table computed (`dcTableCalcGen`), the controller's model now eight states (+ driven wheels' slip speed, boost; six could not hold the slide's slow modes), weights for the right model, gear request, anti-windup, the validation started at the slide's wheel speeds. Validated points AWD 15 / RWD 16 (identified 7–8 / 3–8); AWD corner benchmark 16 corners, 2 given up (circuit), none off; ±4° 72–76 % on the ovals (identified 81–89 %), 48 % on the circuit. Default game unchanged (the autopilot with the controller off drives the same bits as 10b).

- **Done 11 Oct (build 2026-10-11b, see CHANGELOG):** the way into the slide along the family of steady slides (the table at 4–28°, the models scheduled by the slip reference where the family is unbroken). RWD corner benchmark 14 corners, none given up, none off (before: none built); AWD 17, none given up, none off. Drift suite with the controller on 6 of 8 (oval R 83 m 3.14 corrections/s – the default without it 3.06; stage 4711 0.37 s off 800 m after the controller's corner, older logic).

Next (state 12b – the line controller on by default, the drift controller on for RWD):
- **AWD with the drift controller:** the 4711 spot that kept it off is gone with the line controller (0.0 → 0.3 m); left: its slides fail the corner check two times in three (full throttle, the car accelerates past the slide's speed) – check `AP.dcUse` true for AWD with the line controller on the acceptance suite and the drift suite.
- **Speed from the model too:** the line controller steers; throttle and brake still follow the plan with the older rules (off-line cut, running-wide reflex, rear step-out release). A longitudinal part (brakes in the model, the speed reference from the plan) would make the pedals physical as well.
- **The older logic's slides:** the line controller is off while they run (their slide law was tuned on the line law); their exits still swing through (circuit AWD, 9968).
- **The tests measure from the second lap:** the start from rest is seen by no suite (12b: the RWD spin at the circuit's start found in a browser check); `drstat.js` and `drift-bench.js` should judge the first lap as well.
- **Visible corrections:** the line controller makes 1.5–2 per second on its own; the RWD stage 2.9 and the oval 2.9 per second in all – near the bar of 3.

Earlier (state 12a – the controller on for RWD by default):
- **AWD with the controller:** two things keep it off – its slides fail the corner check two times in three (full throttle, the car accelerates past the slide's speed, the slip falls), and stage 4711's spot of the older logic (s ≈ 1647 m: the line law's look-ahead point 2 m to the side where the line crosses the road for the hairpin, the car yaws at 15–20°/s against the line's 4°/s and, braking, the AWD's tail steps out; 0.0 m from the edge without the controller, 0.4 s off with it).
- **The older logic's slide exit** (the target to 0 in ~20 m, the car swings through, ~2 m past the line: circuit AWD s 641, stage 9968 s 57) – the default's closest spots; the controller kept through its exits down to zero slip gave nothing (CHANGELOG 12a).
- **Done in 12b – a path-following law from the car model for the autopilot** (was the proposal): the line law is pure pursuit at 0.36 s × v + 4 m; where the line crosses the road at speed it turns the car at 15–20°/s against the line's 4°/s (4711 s ≈ 1620–1647 m) – shorter or longer look-ahead, yaw damping and edge margins only moved that spot. The slide model's family includes grip: the same LQR machinery on the grip equilibria at the line's radius and speed (feed-forward of the line's curvature, feedback on offset, heading and yaw rate, the driver's delay in the design) would replace the heuristic gains; braking would have to enter the model.
- **The RWD's corners the check removes** (4 of 25 on the benchmark, among them the RWD stage's long corner that the lap holds as one 125 m slide): the build overshoots from a slow approach with slip, or the slide collapses at the plan's speed cap.

Earlier (12 Oct, night – CHANGELOG "How close each case comes to the edge"):
- **Nearest to the default: the controller on with the looser criteria** – acceptance 11 of 11, laps unchanged, 2–5 × the time beyond 12° on the circuit and the stages; open: drift suite stage 19029 AWD 0.3 s off (the spot at s ≈ 687 m, 0.2 m in the acceptance run), and the closest approaches still 0.0 m (circuit AWD s 1273, the RWD stage s 1210 – both older-logic corners the check removed or that are not candidates).
- **The acceptance bar is met narrowly by the default** (two cases 0.0 m from the edge, three within 0.3 m – systematic spots, `test/acc-robust.sh`); with the controller on two of them go over (RWD stage 0.2 s, 4711 0.4 s). Both mechanisms are the older logic's (slide exit swinging through; braking in an S on narrow stages) – patching them only moved the spots (history in CHANGELOG 09h–11b). The way out is the controller taking those corners.
- **The controller on the stages' corners, robust to the approach it meets:** with the looser criteria and no check the RWD holds the RWD stage's long corner as one 125 m slide at 18–23° (1.7 m to spare, 1.5 s faster), but the corner check removes it – from a synthetic approach the build overshoots (31°) or the held slide collapses (throttle to zero, speed 6–13 km/h over the capped reference). The corner benchmark exists now (`test/dc/chkbench.sh`, 12 Oct: RWD 21 of 25 corners kept with the looser criteria, AWD 10 of 26); saturation-aware gains gave nothing (removed), the speed cap off nothing beyond the scatter. Next to try on it: the speed reference consistent with the family (at the capped speed the slip the family has there, rather than the plan's cap against the 22° reference); whether the check predicts the lap (the RWD stage's long corner fails the check in every variant but is held in the lap – the check's lead `AP.tChkLead`).
- **The AWD at full throttle:** the slide's speed grows past the reference, the slip falls (circuit s 1098–1181 m: 19° → 14°, given up) – the AWD's 22° slides at R 85–90 m sit at the throttle's limit.
- **The exit to zero slip by the controller** (it hands over at a 4° target; the older logic's exit swings through) – the family down to grip.

Next (state 11b):
- **Switching `AP.dcUse` on:** acceptance suite with it on **10 of 11** (11 Oct) – only stage 4711 (case k), 0.3 s off at s ≈ 1642 m: braking into a right-hander the older logic's car oversteers to 7° and runs to the road's edge – without the controller it stays on with 0.1 m to spare (lat 3.7 of 3.8 m), with it 3.9 m (the lap differs by a few centimetres; `test/hybrid/spot.js` traces the stretch). A fix belongs to the grip logic (braking with the rear stepping out at the edge), not to the controller. Then the oval's steering corrections (3.14, bar 3 – the default autopilot is at 3.06 there too).
- **The circuit, AWD:** 27 % within ±4° (none given up); **hold quality:** the build overshoots 6–9° and the steering is busy on the rough road (±5° frame to frame).
- **AWD's way into the slide:** its family folds (6–10°), the build jumps the fold with the corner's models – a computed transition (the core's dynamics integrated over the jump) would be the next step there.
- **Drifts on stages – the next large piece** (benchmark: `test/dc/stagebench.sh`; 11 Oct: default criteria 1 corner on five stages, loose criteria 10 corners / 3 given up / none off, 14–18° under the target; the speed reference is now capped by the plan): with corners of ≥ 2 s within 2.5 : 1 the controller takes 3 corners on stage 4711 and 5 on 19029 (none off, one given up), but holds them 14–16° under the target: the stage's corners are short and open fast, the controller engages 1.5–2 s into them (it waits for the corner's radius) and the slide needs 1.4 s to build (a faster reference only overshoots). Long slides on stages need the slide started on the approach (the rotation begun before the radius, as a driver's feint or lift) and the links between consecutive corners (left slide into right slide); the slide model can compute both (the family and its dynamics).
- **Stage 4711's spot** (the acceptance case with the controller on): the older logic's braking into an S at the road's edge; an earlier brake release and a yaw-lag speed plan only moved the excursion (see CHANGELOG 11b).
- **AWD 16°** does not validate above R 30 m (several steady slides near 15° – the family folds); R 120 m plans no slide (R 135 m needs 89 % throttle, above the 80 % reserve).
- Then: drift suite and acceptance with `AP.dcUse` on; snow, sand, tarmac.

Next (before 11a):
- **The steering's 1.3 Hz mode in the hold** (oval R 83 m: 4.2 corrections per second with the controller on, bar 3; outside its corners 1.6): ±10° of command at ~1.3 Hz, unchanged by the yaw-rate or input weights tried, by the deviation limits and by averaging the state. Cause found: a 3 Hz limit cycle through the hands' rate limit (70°/s at the road wheel), which the models do not contain. The steering's change as the input (`DC_DU`, in `dcGain`) cuts it – weight 1000: 3.3 corrections per second in the slide, 2.4 over the lap, 89 % within ±4° – but with it in the whole pipeline the benchmark fell to 27 slides, 4 given up, 2.0 s off (from 37/0/0): it is off (`DC_DU 0`, try with `AP.tDu`) until the validation and the corner check are tuned to it.
- **R 40 m holds 6–11° short of 22°** (±4° 10–46 %); check whether the plan's or the models' operating point is the cause (the slip the R 30/40 m slides settled at: 18–22°).
- **Coverage:** only corners of ≥ 3 s at a radius within 1.6 : 1 qualify – none on 12 random stages, two or three on the circuit, and validation/the corner check remove some per table. Corners that tighten and open need models scheduled along the corner (the exit already keeps the held part's radius) and a start that copes with them.
- **RWD:** the ring validation passes (the regulator holds its slides at steady state) but every corner fails the check – the start: the bumpless start never breaks the rear loose (throttle 0–0.3, yaw reversed); a start with washed-out references built it on the oval R 40 m. A start chosen by trying both in the corner check would need no rule per drive.
- Then switch `AP.dcUse` on: drift suite with it on (the oval's corrections) and the acceptance suite – 11 of 11 already.
- Done 10 Oct: closed-loop measurement of the speed each angle is held at (the table's operating points; `dcPlan`); the way into the slide (entry speed, start on the corner's radius, no older-layer lead-in, the kinematic yaw reference); a drift path along the road's centre through constant-radius corners, blended by curvature.
- Snow, sand, tarmac at the new settings.
- The autopilot computes its command at the physics rate (360 Hz, ≈ 0.4 ms per call); a human decides at 10–30 Hz – deciding at 60 Hz would cut the cost to a sixth with no loss (proposed 9 Oct).

## Braking aid for the player (ideas for later, 9 Oct: the braking point and the release are hard to judge)
The envelope (braking deceleration and lateral grip over speed) and a speed profile give, for the player's own car, the last point to brake for the next corner from the current speed. Ideas, simplest first: (1) a braking marker on the road at that point, a warning zone before it, moving with speed; (2) the line or road coloured green/yellow/red by whether the corner can still be made from here (red = too fast, not merely "braking happens here"); (3) the braking zone as a gradient – strong at the start, fading to the turn-in as the brake should be released for the cornering force (friction circle, the "degression"); (4) a fan of the tightest radii possible at the current speed (v²/lateral grip) drawn from the car, narrowing as it slows; (5) a target dot in the existing g-g diagram (where the plan would be: brake and lateral share); (6) small aids – target speed of the next corner, time to the braking point, 100/50 m boards, a force-feedback pulse, the tutor (U) coaching the braking. Suggested start: (1)+(3) as one road marking, plus (5).

## Later: driving without preprocessing the whole track (last, once everything else works – a step towards autonomous driving)
Today the envelope (per car, surface, setup and driver – not per track), the racing line (optimised over the whole lap) and the speed profile are computed before driving. Ad hoc: the line optimised in a sliding window of 300–500 m ahead every few tenths of a second, the speed profile over the same window (forward/backward pass, longer than the longest braking distance); the envelope stays per car. Expected cost ≈ 0.5–1 % lap time (the line at the window's end does not know what follows).

## Autopilot – race driver (requirement, 7 Oct)

"The fastest lap, using the road (not necessarily the ideal line), going into the limit at a chosen depth and mastering it safely." Done: phase 1 (measured envelope + catch, *Autopilot pace*), phase 2 (racing line by lap-time optimisation), phase 3 (understeer feed-forward, integral on the line error, running-wide reflex; the plan is computed before driving and driven as computed – no learning, by decision). At 100 % all of circuit/stage × tarmac/gravel × AWD/RWD/FWD on the road except circuit gravel FWD 0.3 s.
- Gap between planned and driven lap: 2–3 s at 100 % (circuit tarmac AWD planned 47.8 s, driven 50.3 s). Where it goes (tracking, the catch on RWD exits, the envelope's two lateral points at 40 and 150 m radius) is the next measurement.
- RWD: the catch acts 2–3 s per lap on tarmac at 100 % (power oversteer on exits) – a traction-aware exit (throttle from the rear axle's remaining grip) is the fix, in the plan, not by lowering corner speed.
- If the plan should become more exact, improve the model, not the laps: more lateral points in the envelope (today 40 and 150 m radius), transient load transfer at turn-in.
- The racing line is computed for 100 %; at low pace a different line might be faster – checked against nothing yet.
- Browser: the line search takes 13–20 s of CPU, i.e. about a minute in 5 ms slices the first time per track and setup; afterwards cached.
- Then: drift on the race plan (the drift setting still falls back to the heuristic plan and the ideal line), recovery after an excursion in the catch mode.
- Requirement, 7 Oct: always optimal from the knowledge of the whole track and all parameters – no learning per lap – and able to exceed the limit by a chosen amount.
  - Line at the chosen pace (not only at 100 %): 0.08–0.12 s per lap planned; warm-start from the cached 100 % line, cache per 5 % of pace.
  - Exceeding the limit (Autopilot pace above 100 %): at 110 % no spin, but 0–4.5 s off the road per lap. Needed: a plan that knows it is over the limit – the slide it will be in (slip angle, the grip it costs) and the path it then takes – instead of a speed target the car cannot hold; the first edge guard (speed cut near the edge) made it worse. Then open the slider beyond 100 %.

## Autopilot – next layers
- Base near the limit: done for AWD – phantom clean at 72 % of grip, autopilot with catch at 76 %. Check RWD/FWD at a raised plan.
- Drift: RWD holds 5.2° (gravel) and 6.4–7.8° (snow) of ~8°, AWD only 2.1–2.8°. RWD: more angle runs wide – a law that trades angle against the path (give up angle when the car drifts outward) is the next step. AWD: with the rigid centre, power slides all four wheels; it needs lift-off rotation and left-foot braking to hold the rear out. Tarmac initiation still open (RWD tarmac 3.1°). FWD drift untested.
- Lap-to-lap learning: correct the speed profile where the car ran wide or had reserve (iterative learning control).
- Minimum-time line instead of minimum-curvature.
- Tracking on very tight random stages (up to 6 s off track per lap on tarmac at curves/elevation 60) – recheck with the new plan.
