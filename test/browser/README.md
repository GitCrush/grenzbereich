# Browser tests

Rendering problems that only show in a real browser (a canvas that throws on bad input, compositing, timing)
are checked here with a headless Chromium through Playwright.

    pip install playwright && python -m playwright install chromium-headless-shell
    python probe.py rundkurs gravel 12 out/rk ghost   # per-frame pixel probes in sky and land, saves frames around jumps
    python frames.py etappe gravel 5 out/st           # 24 consecutive frames, for frame-to-frame diffs

`ghost` lets the phantom drive; without it the script holds the throttle and steers in pulses.

The driver view draws the landscape with three.js (r128) from cdnjs. Where the test machine cannot reach the CDN,
`shot3d.py` serves a local copy instead (`npm install three@0.128.0`, or set `THREE_JS` to the file):

    python shot3d.py rundkurs gravel 6 out/circuit.png
