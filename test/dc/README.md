# Drift controller bench (10 Oct)

The tools that built and validated the drift controller in `index.html` (`dcDrive`, `dcTableGen`, see the block before
`driftEqAt`). They drive the engine headless on a ring (`ringX`, R from the command, 30 m to each side).

- `dclib.js` – ring setup, the state x = [β, r, v, e, h, δ] (NS 6; NS 5 without δ, NS 7 with δ̇, NS 10/11 with the wheel
  loads and steering compliance, NS 12 with the body's heave/pitch/roll), least squares (free and structured), the
  discrete Riccati equation (plain iteration), the augmented LQI (`lqiAug`: the driver's delay buffer and lag as states),
  the steady state of a model.
- `ctl.js` – a controller from a model: NS 5 with a model prediction over the delay, or the augmented LQI.
- `ident3.js` – identifies one point: the slide held by a neighbour's controller (`MODEL0`) with per-frame steering
  excitation, smooth road (`SMOOTH=1`), structured fit, gain for the reaction `REACT`. Writes the model JSON (`OUT`).
- `val3.js` – validates a model on its ring: `TSW=1` lets the controller start the slide from straight running at 1 s;
  `SMOOTH=0` is the road as in the game. Prints β mean ± sd against the target and the radial error.
- `chain3.sh DRIVE REACT NS M0 "R:BT …"` – a chain of points, each from the previous model, validated smooth and rough.
- `feat.py dump.json` – which hidden states explain the dynamics (one-step R² with and without wheel loads, compliance…).
- `seed_awd_gravel_r83_b20.json` – the seed built into the game (`DC_SEED`), `model_rwd_gravel_r83_b20.json` its RWD
  counterpart.

Example: `ZB=3 ZE=200 ./chain3.sh awd 0.7 6 seed_awd_gravel_r83_b20.json 65:20 50:20 40:20` – AWD, human reaction,
R 65 → 40 m at 20°. Results of 10 Oct are in the CHANGELOG (build 2026-10-10a).

Added 10 Oct evening (build 10b, see the CHANGELOG):
- `robust.js R b [runs]` – identifies a point several times with other excitation sequences (`AP.dcSeedOff`), inside the
  measurement's world as the game does (`mjSwap`); `COND=n,r,o` adds the rough road and a ring held 3.5 m beside the band
  (loose gravel); `PTS="R:b,…"` several points; `OUT=file.json` writes the models.
- `robust.py models.json [Q=…] [RW=…] [QI=…] [KIN=1] [TAB=0] [V=1]` – the largest closed-loop eigenvalue of every gain
  (from the identification's own condition) on every model of a point; `KIN=1` replaces the path rows by geometry as the
  game now does. This is how the weights `DC_Q`/`DC_RW` were chosen.
- `bench.sh TAG 'SETA' [laps]` – `dc-entry.js` over the ovals R 40/55/83/120 m and the circuit, two lanes; `LS` keeps
  envelope, table and lines per lane (delete when the identification changes).
- `bench3.sh TAG 'SETA' [laps]` – the same over three tables (`dcSeedOff` 0/1000/2000): a corner's outcome depended on the
  table measured, compare variants over all three.
- `chkbench.sh` – the corner check as a benchmark (12 Oct): the controller's candidate corners on five tracks (the RWD and the
  circuit, stages 4711, 9968, 19029) with the looser criteria (`dcRR` 2.5, `dcMinT` 2), RWD and AWD, each driven by the
  corner check (`hybrid/corners.js`); counted: kept, given up, off. `SETA` adds a variant. 12 Oct: RWD 21 of 25 kept,
  AWD 10 of 26.
