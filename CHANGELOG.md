# Changelog

The simulation was developed iteratively; each step below was measured against the test harness before it was kept.

## 1.5.1

- three.js r128 is included in `index.html` instead of loaded from cdnjs: the page is one self-contained file again, makes no request to a third party, and the 3D landscape works offline and from a downloaded copy. The library is unmodified (same SHA-512 as the cdnjs and npm builds), with its MIT licence in a comment above it and in `NOTICE`. `index.html` grows from about 250 to about 880 kB.
- `test/browser/shot3d.py` no longer fails with a syntax error (the route that served a local three.js copy is gone with the CDN).

## 1.5.0

- The landscape of the driver view is a 3D scene in WebGL (three.js r128, loaded from cdnjs with a subresource-integrity hash): a terrain mesh from a gentle height field (14 m grid, 1.6 km beyond the track, lowered under the road); road, shoulders, edge lines, driven band, ruts and snowbanks as ribbon meshes on it; instanced low-poly trees, rocks, bushes and guide posts (up to 14 000 trees, one draw call per model); sky dome, distant mountains, distance fog and a depth buffer, so it is stable from every angle. Without WebGL or without the library the 2D renderer draws the view as before.
- Look: linear lighting with sRGB output; per-surface detail textures (grass and earth, gravel stones, tarmac aggregate, wind-blown snow); steep ground shows bare earth and rock; woodland wherever the forest mask says so; soft high cloud and a sun glow. Trees as composite models with the shading in their vertex colours: spruces of four drooping tiers with snow on the outer branches in winter, broad-leaved trees and birches with lobed crowns, pines, cypresses; light shafts from a low sun. A natural daylight palette (muted, strong aerial perspective) replaces the saturated one.
- 2D renderer: the land is one height field over the whole map instead of polygon bands along the track, ray-marched per screen column; it stops halfway to any other part of the track (on the circuit an infield hillside stood as a tilted sheet in the sky – found in a real browser with `test/browser/`). Track parts outside the road window are drawn far to near, and road, verge, terrain and vegetation fade in instead of appearing out of nowhere. About 25 % less frame time; adaptive detail (Setup → Graphics detail: auto / high / low) no longer changes what is visible.
- Tutor gives one recommendation that keeps the car stable and on the road: it continues the driver's own line (offset and lateral motion, limited to the road), aims a look-ahead down the road from the direction of travel and gives the wheel angle for it – in grip from a steady-state feed-forward calibrated online against the car, in a slide from the front axle's direction of travel against the excess yaw. Measured: a driver steering only by the tutor (0.15 s late, pedals from the phantom's plan at half pace) gets round the circuit and the stage – without leaving the road on tarmac and with AWD on the gravel circuit, 3.4 s off on the gravel stage and 7.2 s with RWD on the gravel circuit (`test/tutornav.js`). On the skidpad it is silent on the line, and a driver who follows it holds the throttle stabs and the power drift that otherwise spin the car (`test/tutortest.js`).
- Key list folded away behind a '? keys' button bottom left (also F1 or '?'); `J` toggles a render debug view.
- New scenario scripts: `tutornav.js`, `strict.js` (a canvas that throws on bad input, as browsers do), `popevents.js` (elements that pop up or build up), `seq.js` (frame strips, holes in the terrain), `prof.js` (frame time); browser tests in `test/browser/`.
- README: driver views from a real browser with the 3D landscape.

## 1.4.0

- Daylight scenery, now the default: per-surface day palettes, clouds at infinity lit on the sun side, a forest treeline on the horizon, meadow and field patches beside the road, barns, sun-lit sides on trees. Graphics mode in View → Graphics and in the track menu, key `I` cycles: day, dusk, and simple – the bare channel view.
- Horizon-locked camera (View → Camera, default; 'car-fixed' restores the windscreen view): the camera follows only 20 % of the grade, 35 % of the cross slope and 40 % of the filtered body motion, so the horizon holds and the bonnet shows the car's attitude – rising on a climb, dropping away on a crest, tilting on a banking. Body pitch and heave reach the camera through a 350 ms filter, roll through 150 ms; the bonnet uses the same filtered motion. Yaw and position are extrapolated by the leftover fraction of a physics step, which removes the jitter of far objects.
- Elevation made visible: the circuit's profile is 1.5× higher (crest 16 m, grades up to 7 %); the Stage has a profile of its own (21 m, grades up to 8 %, three banked sections). The land beside the road is a sidehill with its own relief out to 320 m, so climbs, crests and dips read against the surroundings. Grade, cross slope and vertical curvature are interpolated along the segment instead of stepping at every track point.
- Terrain rendering: the far band is one polygon per side over the whole window (no slices, seams or fans at grazing angles); inner band and verge fade in at the far end of the road window instead of building up segment by segment; offsets on the inside of bends are capped by absolute distance. Vegetation in density waves along the stage, with trunks on all trees, Scots pines and birches (bare in winter) and per-tree colour variation; mountains reduced to one layer of snow-capped peaks.
- Tutor rebuilt and moved to the steering band at the top of the screen. It stabilises the car on its current path and ignores the ideal line, the radius and the drift angle: target = neutral steer θf = atan2(vy + a·r, vx) minus 0.3° per °/s of excess yaw (r − ay/v). It speaks only while a slide grows or unwinds and is quiet in a settled drift; no pedal cues. Measured on the skidpad (`test/tutortest.js`, matrix scenario 21): silent on the line and on a tighter radius; a driver who follows it holds throttle stabs on tarmac and gravel and a power drift in the middle of the pad.
- The phantom's speed plan accounts for elevation: corner speed reduced by the load lost on a crest, crests limited to half the wheel load, braking distances from the energy balance with the height difference. Its driver is a function of the state (`ghostCommand`).
- Lap clock starts when the car first moves after a track selection or reset.
- README screenshots: driver views re-rendered in daylight.

## 1.3.0

- Scenery for the driver view (Optical flow → Scenery): dusk sky gradient and low sun per surface, three ridge layers at infinity (pure rotational flow), aerial haze on road and verge, ground gradient. Landscape per surface: tarmac with snow-capped jagged peaks, cypresses, broad-leaved trees and rock outcrops; gravel with rolling hills, pine, birch and boulders; snow with low hills, snow-laden spruce, snowbanks along the road, a snow-covered road surface with dark polished ruts and snow-coloured ground texture. Vegetation inside the road window is drawn by the road pass itself, far to near, so it no longer pops in and out behind the verge fill. The snowbanks are visual only (see `TODO.md`).
- Tutor overlay (`U`, touch button, Overlays → Tutor): only while the car is becoming unstable (excess yaw rate r − ay/v beyond 9 °/s with the rear axle past its peak, hysteresis out at 3 °/s) it shows the correction on the steering ribbon – a green band from the current wheel position to the target – and one word (counter-steer, lift · counter-steer, unwind). Filtered against road roughness; silent otherwise; display only.
- Default surface is gravel, matching the base setup; changing the surface loads its template (Gravel / Tarmac / new Snow), with the RWD values on top when rear-wheel drive is selected.
- Gauges are damped for the eye (≈100 ms first-order), so wheel loads, usage, hand torque and g readings stop flickering on rough gravel; physics and logger untouched. Cross stripes removed from the driver view again.
- New scenario script `flutter.js`; the harness exports `tutorAdvice`.
- README: driver views re-rendered with the scenery, a snow view added.

## 1.2.0

- Mobile steering: a horizontal slider at the bottom right is now the default, absolute like the mouse (centre is straight ahead, the ends are full lock); the thumb-crank wheel stays available (setup: Touch steering). The first device test found the crank impractical.
- Mobile pedals: the swipe travel adapts to the room the thumb has, so full pedal is always reachable. Start screen and track menu scroll on small screens.
- Automatic gearbox never shifts while the car is sliding (|β| > 6°): an upshift's ignition cut is a lift-off, a downshift's blip and stronger engine braking pulse the rear axle in the middle of the slide.
- RWD drivetrain setup revised: a plated 1.5-way diff at 68/45 % (was 60/40 %), 52 % front roll stiffness, 0.32° rear toe-in, 0.17 rear roll steer and 78 % brake bias. Power-on rotation on tarmac halved (+215 % → +102 % yaw rate on a full-throttle stab at 0.45 g), and the inside rear stays on the ground at the limit (the matrix INFO for RWD on tarmac is gone). Measured with `test/rwdmid.js` and `test/rwdsetup.js`.
- Changing the drivetrain now syncs the whole setup panel, brake bias included.
- New scenario scripts: `rwdmid.js` (RWD setups between neutral and forgiving), `rwdcatch2.js` (catching a power slide with human reaction time and hand speed), `slideosc.js` (oscillations in a big slide).
- README screenshots re-taken: the setup panel shows the touch-steering choice.

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
