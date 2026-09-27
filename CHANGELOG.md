# Changelog

The simulation was developed iteratively; each step below was measured against the test harness before it was kept.

## 1.1.0

- Gamepad: a moving stick takes the steering back from the mouse, as the arrow keys do.

- Mobile controls rebuilt: swipe zone on the left for throttle and brake, thumb-crank steering wheel on the right (ratio and return-to-centre in the setup), handbrake bottom left, gears and view top right. Tilt steering is now optional and asks for motion permission only when switched on.

- Keyboard throttle metering as a driver model (setup: Keyboard throttle metered / raw): the key eases (floor 35 %) when the driven wheels spin beyond 1.5× the peak slip while the car is going straight; off the line and once sideways (|β| > 6°) the key is raw, so a held slide keeps its wheelspin and its revs. On a skidpad a rear-wheel-drive car no longer steps out the moment the automatic shifts out of the limiter (β 8° instead of a spin).

- Axle differentials rebuilt as plated Coulomb constraints applied after the wheel integration, with the conventional locking definition (the torque difference the diff can hold as a share of the input torque). The previous viscous bias needed a speed difference before it acted and reported half the conventional value; 55 % on the old slider was ≈110 % conventional. New defaults: AWD/FWD 40/25 %, RWD 60/40 % (drive/coast), measured with `test/lsdsweep.js` and `test/awdlsd.js`.
- Setup templates Gravel / Tarmac / RWD drift with a full slider sync (`SETUP_TEMPLATES`, `applySetupTemplate`, `syncSetupUI`); measured in `test/tpltest.js`. The Tarmac template keeps the inside rear on the ground at the limit (263 N at 1.23 g).
- Automatic gearbox shifts at the limit after a short hold with a deeper throttle ease instead of holding the gear indefinitely.
- Matrix: launch scenarios use a traction-metering driver; the skidpad-upshift scenario replaces the usage rule.
- README screenshots re-taken with the new setup panel.

## 1.0.2

- Mouse steering holds its last position when the pointer leaves the view, so overshooting onto the side panel or out of the window no longer drops the steering in the middle of a slide.
- The mouse travel is fitted to the window so that full lock is reachable at its edges; moving the travel slider switches the fit off.
- The arrow keys (and A / D) take the wheel back from the mouse.
- Test harness: the DOM stubs provide a window size.

## 1.0.1

- Renamed to Grenzbereich (formerly The Limit).
- Mouse steering by hover only: the page no longer captures the pointer. The mouse steers while it is over the view; throttle stays on the keyboard. The `X` key (centre mouse) is gone with it.
- README screenshots taken in a browser with the full interface (`test/uishot.js`).

## 1.0.0 — first public release

**Physics**
- Roll stiffness split into fixed wheel rates and anti-roll bars derived from a roll-gradient setting; geometric load transfer per axle from roll-centre heights.
- Damper curves with knee and bump/rebound asymmetry; progressive bump stops and hard droop stops per wheel; inelastic safety clamps.
- Static toe and bump steer (roll steer follows); camber coefficient 0.25 and camber-dependent peak grip.
- Plated axle differentials with separate drive and coast lock; rigid centre coupling as a momentum-conserving constraint, plated below 100 %; handbrake disconnects the rear axle with a 0.3 s re-engagement.
- Sequential gearbox: ignition-cut upshifts with the clutch engaged, blip downshifts; the automatic no longer hunts while the wheels spin.
- Boost as a spool state that cannot produce torque with the throttle closed.
- Tyre: separate longitudinal and lateral relaxation lengths; force direction blended towards the sliding direction beyond the peak; wedge/plough term with Bekker share and saturating dynamic share.
- Road-fixed reference frame with grade, cross slope and vertical curvature; elevation profile for the circuit.
- Road state: swept band around the driven line, loose material and berm, ruts with wall forces.

**Input**
- Setup per drivetrain: selecting RWD loads its own rear-axle values (diff lock, coast lock, roll distribution, rear toe, roll steer).
- Automatic gearbox as a driver model: waits up to half a second for the corner exit when the driven axle is loaded, then shifts with a throttle ease so the shift kick does not spin the driven wheels; at the limit it holds the gear.

**Driver models for input devices**
- Clutch driver model with launch rpm and stall protection.
- Keyboard threshold braking.
- Phantom plans with the local grip and meters traction.

**Instrumentation**
- Consistency matrix (106 scenarios) and reference benchmark in `test/`.
- Incident detector catches spins with both axles saturated; axle usage is load-weighted.
- On-screen diagnosis when the car cannot move (handbrake, brake, clutch, wheels off the ground).

**Rendering and sound**
- Driver view with elevation: far-to-near road pass, terrain silhouettes, camera following the road plane; top view with relief shading and elevation profile strip.
- Gear whine, anti-lag bangs, limiter cut, shift cut, stone hits, bump-stop thumps.
