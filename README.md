# FishBite

Electronic bite detector: ESP32 + MPU6050 on the rod tip, BLE, buzzer.

- `FishBite.ino`: deployed device. Self-calibrates, alerts on bites, sends a fight-duration summary. No setup, ever.
- `FishBite_Logger` + `tools/`: one-time dev tool to validate the detector against labeled data. Not used while fishing.

## Why amplitude isn't enough

Wind, waves, and bites differ in **shape**, not size:

| source | character | timescale |
|---|---|---|
| wind tilt | held lean | seconds, near-DC |
| wave bob | periodic | 0.2–1 Hz |
| bite | sharp, one-off | 50–200 ms |
| fight | severe held bend, thrashing | tens of seconds to minutes |

## Research

- **Test curve** (rod bend rated at 90° load): informed `FIGHT_TILT_DEG=45°`, between casual lean (~20°) and rated bend (90°).
- **Angling convention**: hookset angle, fight angle, and rod-holder angle guides independently converge on ~45° as the practical working angle, corroborating the test-curve-derived figure from a completely different kind of source.
- **Rod action/power classes**: industry-standard comparison tools (e.g. the CRB deflection chart) show rods vary by design, extra-fast rods bend the top 15-20% of the blank, fast ~25%, moderate ~50%, slow/parabolic more than that. This means `FIGHT_TILT_DEG=45°` is tuned for a typical fast/moderate-action rod, not a universal constant; a different action class would likely need retuning. No published source gives a clean formula for that conversion, so this isn't guessed at further.
- **US20090158635A1** (bite-detector patent): combines angle-change and angular-rate, the same hybrid used here.
- **Taniguchi et al. 2025** (IEEJ): frequency-domain bite detection is published precedent for `detect_periodicity_reject`.
- **Fish fight duration**: highly species-dependent (bass under 2min, sturgeon 45min to 2hr), so no "normal" duration is assumed.

Domain grounding, not validation. Thresholds are still unproven until scored against real data.

## Repo layout

| path | what it is |
|---|---|
| `FishBite.ino` | deployed detector: bite alerts + fight tracking over BLE |
| `FishBite_Logger/FishBite_Logger.ino` | logging-only: raw IMU + BLE ground-truth markers |
| `tools/analyze.py` | parses/plots captures, replays detectors causally, scores them |
| `tools/make_synthetic_capture.py` | generates a synthetic capture for pipeline testing |
| `examples/synthetic/` | example captures + plots + scores |

## Marking

Dev/validation only: a phone writes `1` (bite) or `2` (false trigger) to a BLE characteristic. Markers can lag by seconds, so `score()` uses a wide backward-biased window (15s back / 2s forward).

## Status

Firmware and all four detectors (`amplitude`, `highpass`, `jerk`, `periodicity_reject`) work. Tested end-to-end on synthetic data: all 3 planted bites, zero false positives, both tackle weights. **No real field session yet.** Fight-tracking thresholds are grounded in published rod mechanics and converging angling convention (see Research), tuned for a fast/moderate-action rod specifically. The four detectors' thresholds are still starting guesses, unrelated to that research; no field data exists yet to tune them.

## Limits

- Zero real capture sessions; one rod, one rig.
- Marker lag is seconds, not milliseconds, handled by a wide window.
- Fight-tracking thresholds are research-grounded but still unvalidated, and tuned for one rod action class. The four detectors' thresholds are unvalidated guesses. Neither has been scored against real data.
- Rod tip range of motion (~20°+) is a hands-on estimate, depends on tackle weight.
