# Performance: pre-port vs standalone vs hdtSMP64

Measured 2026-11-08, one session per game, one save, one scene: SkyrimVR 1.4.15 (null driver, no headset)
and Skyrim SE 1.7.104. Raw rows: [VR](perf-data/smp_ab_results.json), [SE](perf-data/smp_ab_se_results.json);
the harness is [perf-data/smp_ab.py](perf-data/smp_ab.py).

## What was compared

| Arm | What simulates the hair |
|---|---|
| `hdt` | hdtSMP64 alone, with both GPU solvers off |
| `pre` | Open Shaders' in-tree SMP feature (branch `claude/hair-body-capsules` at `1a993b31f5`, `ALL-TRACY` AIO, DLL `7CE718E25DC3C3E3`) |
| `port` | this plugin (VR run: DLL `352ABEAC24A13D7B...`; SE run: DLL `9C30216D3C56A10C...`, which differs only by the timer thread fix described below, no solver change) |

All three were loaded together and switched at runtime, so the scene, camera and settings are identical.
Each crowd size ran `hdt`, `pre`, `port`, `hdt` in that order, 6 s settle then a 15 s window.
The crowd is the test-crowd spawner's wigged characters (10 bases repeated up to 30). A warm-up pass
with 5 characters switched each arm on once first, so shader compiles are not in the windows.

## Throttling

Every throttle was lifted for all arms, so no arm simulates fewer actors than another:

- hdtSMP64 `configs.json`: `autoAdjustMaxSkeletons` false, `maximumActiveSkeletons` 1000, `budgetMs` 1000,
  `minCullingDistance` 100000, `minScreenSizePercent` 0 (the original file was restored afterwards).
- This plugin and the in-tree feature: `MaxClaims` and `MaxSimulatedClaims` 256, `SimulationDistance` 20000,
  `MinScreenSizePercent` 0.

## Results

Frame time (ms, `hdt` is the mean of its two windows). The null driver caps the frame at about 31 ms
(32.3 fps), so the arms are indistinguishable here and the table cannot show a frame-time win or loss.

| Crowd | Claims | hdt | pre | port |
|---|---|---|---|---|
| 0 | 9 | 31.00 | 31.01 | 31.00 |
| 5 | 16 | 31.02 | 31.00 | 31.00 |
| 10 | 21 | 31.12 | 31.38 | 31.01 |
| 20 | 33 | 31.03 | 31.15 | 31.07 |
| 30 | 38 | 31.00 | 30.98 | 31.00 |

GPU milliseconds per frame, port (timestamp queries, 484 frames per window at crowd 30). "Solver total"
adds the non-nested passes (`RigidContact` already contains `ContactPasses` (once `ChainContact`) and its three sub-passes).

| Crowd | RigidContact | RigidSolve | RigidSetup | SurfacePrep | Solver total | Share of VR 11.1 ms |
|---|---|---|---|---|---|---|
| 0 | 0.25 | 1.32 | 0.88 | 0.10 | 2.75 | 25% |
| 5 | 1.97 | 1.44 | 0.73 | 0.21 | 4.51 | 41% |
| 10 | 3.38 | 1.65 | 0.80 | 0.19 | 6.19 | 56% |
| 20 | 7.29 | 1.74 | 0.71 | 0.10 | 10.09 | 91% |
| 30 | 8.03 | 1.84 | 0.66 | 0.10 | 10.91 | 98% |

Cross-check at crowd 30 against Tracy on the `pre` arm (793 frames, 38 claims), ms per frame:
`RigidContact` 7.88 (port 8.03), `ContactSpheres` 7.69 (7.72), `RigidSolve` 1.92 (1.84),
`RigidSetup` 0.66 (0.66), `SurfacePrep` 0.21 (0.10), `ChainSolve` 0.04 (0.04). The port matches
the pre-port build pass for pass, which also validates the new timer.

## SE (frame time is not capped)

SE ran at 74 fps with no crowd, so frame time separates the arms. Same arms, order, throttles and windows as
VR. Frame ms; `hdt` is the mean of its two windows and the spread is the gap between them (the noise floor).

| Crowd | Claims | hdt (spread) | pre | port | port vs hdt | Port GPU solver ms (share of 16.67) |
|---|---|---|---|---|---|---|
| 0 | 14 | 13.49 (0.08) | 13.07 | 13.32 | -0.16 (-1%) | 1.63 (10%) |
| 5 | 26 | 16.20 (0.79) | 15.22 | 15.20 | -1.01 (-6%) | 2.18 (13%) |
| 10 | 36 | 18.55 (0.43) | 17.09 | 17.12 | -1.43 (-8%) | 2.83 (17%) |
| 20 | 58 | 26.72 (0.08) | 25.95 | 25.81 | -0.91 (-3%) | 8.84 (53%) |
| 30 | 79 | 30.82 (0.42) | 30.91 | 30.95 | +0.13 (0%) | 10.07 (60%) |

- **Port equals pre-port** at every size (within 0.25 ms), as on VR.
- **Against hdtSMP64 both GPU solvers are 3-8% faster at 10 and 20 characters**, a gap larger than the
  hdt-to-hdt spread at those sizes; at 5 it is about the size of the noise, at 0 and 30 there is no difference.
- At 30 characters all three arms land on 30.6-31.0 ms (32.3 fps), the same figure the VR null driver capped
  at. The cause was not identified, so this size says nothing about the arms.
- Frame generation and NR state were not read back on SE either.

## What this does and does not show

- **Port equals pre-port.** Same passes, same cost, no regression from moving out of the tree.
- **No frame-time result on VR.** The frame is pinned at the null-driver cap, so the GPU headroom hides the solver.
  The 98% share is the cost against the 11.1 ms budget a real headset needs, not a measured slowdown.
- **hdtSMP64 comparison is from SE only**, where the frame is not capped (table above): a modest gain at
  10-20 characters, none at 0 and 30.
- A first SE run crashed in the driver when the timer was switched off from the devbench thread. The timer now
  only raises a flag there and the render path does the work; the rows above are from the rerun with the fix.
- **Contact spheres dominate** the GPU cost from about 10 characters, which is where optimisation should go.
- `totalStepMicroseconds` is a single last-frame sample (pre 57-409 us, port 176-628 us) and varies both
  ways between sizes, so it is not a CPU comparison.
- Frame generation and NR state were not read back (`frameGenActive` was null), and the render resolution
  was not recorded.
