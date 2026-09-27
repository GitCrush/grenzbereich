# Grenzbereich — Rally Vehicle Dynamics

A browser-based rally driving simulator built as a **training tool for the grip limit**, not as a racing game. *Grenzbereich* is German for the region at the limit of grip, which is what the simulator is built around. One HTML file, no dependencies, no assets: a Rally2-class car (1230 kg, 1.6-litre restricted turbo, five-speed sequential, rigid centre coupling) on tarmac, gravel and snow, driven with mouse, keyboard, gamepad, tilt or a steering wheel.

Every mechanism in the physics is written down in the code and measured against standard manoeuvres. A phantom car drives the same physics on the ideal line, a data logger names the cause of every spin, and a headless test harness runs 100+ scenarios against plausibility rules.

**[Drive it here](https://gitcrush.github.io/grenzbereich/)** — desktop browser recommended; move the mouse over the view to steer.

The simulator is the single file [`index.html`](index.html) in this repository, served by GitHub Pages at [gitcrush.github.io/grenzbereich/index.html](https://gitcrush.github.io/grenzbereich/index.html). Nothing to install: open that address, or download the file and open it locally.

![Driver view on gravel, sliding with the setup panel and telemetry open](docs/screenshots/driver-view-gravel.png)

| Banked tarmac corner near the limit | Top view: tyre forces, dust, skid marks, phantom and elevation profile |
|---|---|
| ![](docs/screenshots/driver-view-banked-tarmac.png) | ![](docs/screenshots/top-view-gravel.png) |

## What it is for

The interesting part of driving a rally car happens in the last ten percent of grip: where the front axle starts to push, where the rear steps out, where lifting the throttle rotates the car and where a locked wheel stops steering. Grenzbereich is built around making that region legible:

- **Tyre usage per axle** in the HUD, load-weighted, colour-coded from the torque peak to the grip limit.
- **A phantom** that drives the same car on the ideal line with a consistent driver model, so you can see where it brakes and how much it slides.
- **A data logger** (14-second ring buffer) with an incident detector that freezes the window around a breakaway and names the probable cause — lift-off, locked wheels, power-on, entry speed.
- **Exercises**: skidpad, slalom, chicane, hairpins, sweeper, figure eight, braking box.
- **Instrumented views**: a top view whose zoom covers your braking distance, and a driver view built as a flow field (near-field texture, guide posts, road edge, vehicle axis vs. velocity vanishing point — the horizontal distance between the two *is* the slip angle).

## Physics model

The model is a textbook multi-body vehicle model rather than a soft-body or a game-tuned handling model. It runs at 360 Hz with a semi-implicit Euler integrator.

**Tyre** — normalised combined-slip characteristic with separately parameterised rise and drop (peak at unit normalised slip, residual friction `tail`, drop width `fall`), load sensitivity (−6.5 % per kN), degressive cornering stiffness (∝ Fz^0.65), sliding-velocity dependence, camber as slip-angle offset and as peak-grip modifier, separate longitudinal (0.15 m) and lateral (0.45 m) relaxation lengths, and a blend of the force direction towards the true sliding direction beyond the peak. On loose surfaces a **plough/wedge term** (Bekker-style bulldozing plus a saturating ρ·A·v² dynamic share) provides the lateral support and the yaw damping that make gravel driveable sideways.

**Vertical dynamics** — sprung body (heave, pitch, roll) over four unsprung masses with tyre springs; fixed wheel rates per axle (1.7 / 2.1 Hz), anti-roll bars derived from a roll-gradient setting and a distribution slider, damper curves with knee and bump/rebound asymmetry, progressive bump stops and hard droop stops per wheel, roll centres and anti-dive/squat as geometric load transfer.

**Kinematics** — Ackermann share, steering compliance with pneumatic and mechanical trail (the hand torque drives the steering-wheel display and force feedback), camber gain, static toe and bump steer (roll steer follows from it).

**Drivetrain** — engine as its own inertial state with a torque map and restrictor plateau, turbo spool as a first-order state (with anti-lag), stick-slip clutch with a driver model for launch and stall protection, five-speed sequential with ignition-cut upshifts and throttle-blip downshifts, plated axle differentials as Coulomb constraints with separate drive and coast locking values in the conventional sense (the torque difference the diff can hold as a share of the input torque), and a **rigid centre coupling** (Rally2 has no centre differential) implemented the same way. The handbrake disconnects the rear axle from the drive, as the real car's hydraulic clutch does.

**Road** — a road-fixed reference frame carries grade, cross slope and vertical curvature: crests unload the car, dips load it, banking presses it into the road. The circuit has an elevation profile with a blind crest, a compression, a banked corner and an off-camber one. The **road state** model makes the surface non-uniform across its width: a swept band around the driven line, loose material outside it, a berm at the edge, and ruts whose walls act on the wheels through the local cross slope.

**Driver models for the input device** — a keyboard key is on or off, a foot is not. The keyboard brake holds the pedal at the lock point (threshold braking, ~5 Hz), the keyboard throttle backs off when the driven wheels spin beyond the tyre's peak slip and comes back as they hook up (launches excepted), the clutch is operated by a launch-rpm model, and the keyboard steering commands the front slip angle rather than the wheel angle, so counter-steer follows by itself. Each of these can be switched to raw in the setup. Analogue inputs (pad, wheel) stay raw. Mouse steering follows the pointer position: the horizontal offset from the centre of the view is the steering-wheel angle, linear, no self-centring, the travel fitted to the window so that full lock is reached at its edges; once the mouse has taken the wheel it keeps it wherever the hand goes, until a steering key or a moving gamepad stick takes it back. Throttle stays on the keyboard, the mouse buttons are clutch and brake.

## Measured behaviour

From the test harness, all-wheel drive, default setup (tarmac / gravel / snow):

| Manoeuvre | Value | Rally2 reference |
|---|---|---|
| 0–100 km/h | 3.8 / 5.1 / 7.4 s | 4.3–5 s tarmac, ≈5 s gravel |
| 100–0 km/h, no ABS | 36.6 / 62.3 / 97.9 m | 32–36 m tarmac, 50–60 m gravel |
| Steady-state lateral limit | 1.25 / 0.87 / 0.53 g | 1.3–1.4 g tarmac, 0.7–0.9 g gravel |
| Understeer gradient | 1.5 / 3.0 / 5.0 °/g | ≈1–2 °/g tarmac |
| Roll gradient | 3.2 °/g | 3–5 °/g gravel setup |
| Yaw response t90 at 80 km/h, 0.4 g | 0.11 / 0.19 s | 0.10–0.20 s |
| Top speed | 199 km/h at the limiter in fifth | 190–200 km/h |

Every merged change must keep `test/consistency.js` clean: 108 scenarios across three drive types and three surfaces, checking ordering, left/right symmetry, physical invariants (ΣFz = m·g, ay = v·r, forces inside the friction circle, deceleration ≤ µ·g, acceleration ≤ P/(m·v)), recovery from slides, handbrake behaviour, determinism and state separation between player and phantom.

## Controls

| | Keyboard | Mouse (over the view) | Gamepad |
|---|---|---|---|
| Steer | ← → / A D | pointer position | left stick |
| Throttle / brake | ↑ ↓ / W S | keyboard / RMB | RT / LT |
| Clutch | Shift | LMB | LB |
| Handbrake | Space | | A |
| Gears | E / Q | | bumpers |

`R` reset · `B` back on track · `T` track menu · `O` setup · `V` top/driver view · `G` phantom · `Z` replay · `L` export CSV · `P` short excerpt · `H` hide HUD · `M` sound · `C` cycle views (driver, top, top north-up) · `+ −` zoom.

On phones and tablets (landscape): swipe up on the left half for throttle and down for brake – the travel from where the thumb landed sets the pedal – and crank the steering wheel on the right with the thumb; it stays where you leave it (or returns to centre, see setup). Handbrake bottom left, gears and view top right. Tilt steering remains available as an option. Logitech wheels get constant-force feedback through WebHID in Chromium browsers.

## Setup

Three templates load complete, measured setups: **Gravel** (the base: soft, rear-biased roll stiffness, 76 % front brake bias, diff locks 40/25 %), **Tarmac** (stiffer and lower, 52 % front roll stiffness so the inside rear stays on the ground at the limit, 82 % front brake, locks 40/25 %) and **RWD drift** (rear-wheel drive with a strong rear diff, 75/50 %, and a stable rear axle). Selecting a drivetrain loads its rear-axle values on top (rear-wheel drive: locks 60/40 %, 48 % front roll stiffness, 0.25° rear toe-in).

The setup panel (`O`) exposes the quantities a rally team would adjust: surface, drive type, centre lock and split, axle lock on drive and coast, brake bias, roll distribution and roll gradient, static camber, toe, rear roll steer, damping, CoG height, ABS and traction control as training aids, anti-lag, keyboard pedal travel and threshold braking, steering mode, road state, elevation profile, and the visual channels of the driver view.

## Known limitations

- The tyre is a normalised model without temperature, pressure or wear, and its parameters are engineering estimates, not measured data.
- One car, one circuit with elevation, a stage loop and seven practice layouts. No damage, no collisions with the surroundings.
- The phantom's driver model is tuned for all-wheel drive; as a front- or rear-wheel-drive car it is slower and occasionally runs wide on gravel.
- The road state is static: the swept band and the ruts follow the ideal line and do not yet evolve with your own passes.
- The sound is synthesised in the browser and has not been compared with recordings.

## Test harness

The physics runs headless in Node against DOM stubs. See [`test/README.md`](test/README.md).

```
cd test
python3 extract.py          # pulls the script block out of ../index.html
node consistency.js         # the scenario matrix, 2–3 minutes
node bench.js               # the reference figures
```

With `@napi-rs/canvas` and `node-web-audio-api` installed, `shot.js` renders the driver view without a browser and `soundtest.js` renders sound demos as WAV — useful for reviewing changes. The README screenshots come from `uishot.js`, which drives the page in a headless Chromium through Playwright, with the phantom's driver model at the wheel.

## Contributing

Physics changes are welcome when they come with a scenario in the harness and a rationale in the code. The comment style is deliberate: every mechanism states what it models, why it is there and what breaks without it. The single-file structure is kept for distribution; a module split with a build step is the first structural change planned.

Open work is listed in [TODO.md](TODO.md). Good first topics: dynamic road state (each pass sweeps the line and deepens the ruts — DiRT Rally 2.0's 150-step degradation is a useful calibration reference), a phantom driver model for two-wheel drive, a tarmac setup preset, tyre temperature.

## Licence

Apache License 2.0 — see [LICENSE](LICENSE). The design decisions and their justification are documented in the code.
