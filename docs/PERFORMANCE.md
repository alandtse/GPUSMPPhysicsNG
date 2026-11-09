# Performance: pre-port vs standalone vs hdtSMP64

Measured 2026-11-08 on SkyrimVR 1.4.15 (null driver, no headset), one session, one save, one scene.
Raw rows: [perf-data/smp_ab_results.json](perf-data/smp_ab_results.json).

## What was compared

| Arm | What simulates the hair |
|---|---|
| `hdt` | hdtSMP64 alone, with both GPU solvers off |
| `pre` | Open Shaders' in-tree SMP feature (branch `claude/hair-body-capsules` at `1a993b31f5`, `ALL-TRACY` AIO, DLL `7CE718E25DC3C3E3`) |
| `port` | this plugin at `4e40505` (DLL SHA-256 `352ABEAC24A13D7B...`) |

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
adds the non-nested passes (`RigidContact` already contains `ChainContact` and its three sub-passes).

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

## What this does and does not show

- **Port equals pre-port.** Same passes, same cost, no regression from moving out of the tree.
- **No frame-time result.** The frame is pinned at the null-driver cap, so the GPU headroom hides the solver.
  The 98% share is the cost against the 11.1 ms budget a real headset needs, not a measured slowdown.
- **No hdtSMP64 comparison.** hdtSMP64 runs on the CPU and its cost does not appear in a capped frame.
  The CPU-side comparison on record is [project-hair-physics-actor-scaling-vs-smp-2026-10-18] in gbrain.
- **Contact spheres dominate** the GPU cost from about 10 characters, which is where optimisation should go.
- `totalStepMicroseconds` is a single last-frame sample (pre 57-409 us, port 176-628 us) and varies both
  ways between sizes, so it is not a CPU comparison.
- Frame generation and NR state were not read back (`frameGenActive` was null), and the render resolution
  was not recorded.
