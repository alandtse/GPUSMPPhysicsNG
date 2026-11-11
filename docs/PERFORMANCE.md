# Performance: pre-port vs standalone vs hdtSMP64

Measured 2026-11-08, one session per game, one save, one scene: SkyrimVR 1.4.15 (null driver, no headset)
and Skyrim SE 1.7.104. Raw rows: [VR](perf-data/smp_ab_results.json), [SE](perf-data/smp_ab_se_results.json);
the harness is [perf-data/smp_ab.py](perf-data/smp_ab.py).

## What was compared

| Arm    | What simulates the hair                                                                                                                                          |
| ------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `hdt`  | hdtSMP64 alone, with both GPU solvers off                                                                                                                        |
| `pre`  | Open Shaders' in-tree SMP feature (branch `claude/hair-body-capsules` at `1a993b31f5`, `ALL-TRACY` AIO, DLL `7CE718E25DC3C3E3`)                                  |
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

| Crowd | Claims | hdt   | pre   | port  |
| ----- | ------ | ----- | ----- | ----- |
| 0     | 9      | 31.00 | 31.01 | 31.00 |
| 5     | 16     | 31.02 | 31.00 | 31.00 |
| 10    | 21     | 31.12 | 31.38 | 31.01 |
| 20    | 33     | 31.03 | 31.15 | 31.07 |
| 30    | 38     | 31.00 | 30.98 | 31.00 |

GPU milliseconds per frame, port (timestamp queries, 484 frames per window at crowd 30). "Solver total"
adds the non-nested passes (`RigidContact` already contains `ContactPasses` (once `ChainContact`) and its three sub-passes).

| Crowd | RigidContact | RigidSolve | RigidSetup | SurfacePrep | Solver total | Share of VR 11.1 ms |
| ----- | ------------ | ---------- | ---------- | ----------- | ------------ | ------------------- |
| 0     | 0.25         | 1.32       | 0.88       | 0.10        | 2.75         | 25%                 |
| 5     | 1.97         | 1.44       | 0.73       | 0.21        | 4.51         | 41%                 |
| 10    | 3.38         | 1.65       | 0.80       | 0.19        | 6.19         | 56%                 |
| 20    | 7.29         | 1.74       | 0.71       | 0.10        | 10.09        | 91%                 |
| 30    | 8.03         | 1.84       | 0.66       | 0.10        | 10.91        | 98%                 |

Cross-check at crowd 30 against Tracy on the `pre` arm (793 frames, 38 claims), ms per frame:
`RigidContact` 7.88 (port 8.03), `ContactSpheres` 7.69 (7.72), `RigidSolve` 1.92 (1.84),
`RigidSetup` 0.66 (0.66), `SurfacePrep` 0.21 (0.10), `ChainSolve` 0.04 (0.04). The port matches
the pre-port build pass for pass, which also validates the new timer.

## SE (frame time is not capped)

SE ran at 74 fps with no crowd, so frame time separates the arms. Same arms, order, throttles and windows as
VR. Frame ms; `hdt` is the mean of its two windows and the spread is the gap between them (the noise floor).

| Crowd | Claims | hdt (spread) | pre   | port  | port vs hdt | Port GPU solver ms (share of 16.67) |
| ----- | ------ | ------------ | ----- | ----- | ----------- | ----------------------------------- |
| 0     | 14     | 13.49 (0.08) | 13.07 | 13.32 | -0.16 (-1%) | 1.63 (10%)                          |
| 5     | 26     | 16.20 (0.79) | 15.22 | 15.20 | -1.01 (-6%) | 2.18 (13%)                          |
| 10    | 36     | 18.55 (0.43) | 17.09 | 17.12 | -1.43 (-8%) | 2.83 (17%)                          |
| 20    | 58     | 26.72 (0.08) | 25.95 | 25.81 | -0.91 (-3%) | 8.84 (53%)                          |
| 30    | 79     | 30.82 (0.42) | 30.91 | 30.95 | +0.13 (0%)  | 10.07 (60%)                         |

- **Port equals pre-port** at every size (within 0.25 ms), as on VR.
- **Against hdtSMP64 both GPU solvers are 3-8% faster at 10 and 20 characters**, a gap larger than the
  hdt-to-hdt spread at those sizes; at 5 it is about the size of the noise, at 0 and 30 there is no difference.
- At 30 characters all three arms land on 30.6-31.0 ms (32.3 fps), the same figure the VR null driver capped
  at. The cause was not identified, so this size says nothing about the arms.
- Frame generation and NR state were not read back on SE either.

## Rerun on 2026-11-09 (SE, today's code against hdtSMP64)

The tables above predate the removal of the chain solver and the in-tree arm. This run is SE 1.7.104, DLL
`05CFBFF3B98C79DE`, hdtSMP64 against this plugin only (`hdt`, `port`, `hdt`), same crowd, throttles lifted as above,
6 s settle then a 15 s window, sizes run in separate sessions of under 5 minutes. Raw rows:
`perf-data/smp_ab_se_results_2026-11-09_*.json`. Frame ms; `hdt` is the mean of its two windows.

| Crowd | Claims | hdt (spread) | port  | port vs hdt   |
| ----- | ------ | ------------ | ----- | ------------- |
| 0     | 14     | 24.63 (0.14) | 22.33 | -2.30 (-9%)   |
| 5     | 26     | 27.84 (0.08) | 26.63 | -1.21 (-4%)   |
| 10    | 36     | 31.10 (0.16) | 26.69 | -4.41 (-14%)  |
| 20    | 58     | 41.73 (0.07) | 30.45 | -11.28 (-27%) |
| 30    | 80     | 48.12 (1.20) | 34.37 | -13.75 (-29%) |

- **The machine ran slower than on 2026-11-08** (hdtSMP64 alone 24.6 ms with no crowd against 13.5 ms then), so compare
  within this table only, not against the table above. The cause was not identified.
- **Port GPU ms per frame at crowd 30:** `ContactSpheres` 1.17 (two calls), `RigidSolve` 1.10, `RigidSetup` 0.65,
  `SurfacePrep` 8.88. The `SurfacePrep` figure is a wait, not solver work: on SE it sat at 8 to 9 ms for every crowd
  above 0 (0.04 at crowd 0), all of it in the 10 BVH refit dispatches. Ten empty dispatches of the same shader cost
  0.014 ms, GPU skinning and the batched palette made no difference, and replacing the 10 dispatches with one thread
  group (correct on `surfaceCheck`, 0 mismatches) cut `SurfacePrep` to 0.04 ms but moved the same 8 to 11 ms into
  `ContactPasses` and left the frame at 30.9 ms against 30.5 ms. So the SE per-pass timestamps charge the first pass that
  has to wait for the GPU to catch up with the game's rendering, and the solver's own cost is the 3 ms the other passes
  add up to. The single-group change was reverted. The cause of the wait itself was not identified.
- **Tracy on SE, 2026-11-10 (crowd 20, `xmake f --tracy=y`, client v0.15.0 to match the profiler server):** the
  `SurfacePrep` 8 to 9 ms is one dispatch. With a GPU zone per dispatch, refit level 3 takes 8.6 ms and the other nine
  levels, the bone, pose, triangle and leaf passes each take 0.005 ms. A zero-work sentinel pass placed just before
  `SurfacePrep` costs 0.000 ms with no gap after it, so the wait is not earlier game work draining. Level 3 starts
  about 3.9 ms after the frame mark and always ends 12.6 ms after it (stdev 0.7 ms), so the GPU resumes at a fixed
  point in the frame. The shader is a 64-thread min/max of two child boxes. The cause is still unknown; an ETW GPU
  trace needs an elevated shell. Tracy GPU zones are D3D11 timestamp queries, so they show duration, not queue waits.
  Captures: `F:	mp	racy_se_20261110_baseline.tracy`, `..._per_dispatch.tracy`.
  A follow-up with VR confirmed not running (`..._level3_experiment.tracy`): a zero-work pass right before level 3
  costs 0.000 ms, level 3 takes 9.0 ms, and the same dispatch issued again straight after takes 0.004 ms. The refit
  levels hold only 64, 32, 16, 8, 4, 2 and 1 entries, so the work is not the cause. It is a once-per-frame wait that
  starts at level 3 and ends at a fixed point in the frame. The earlier single-group refit moved it to `ContactPasses`,
  so it follows the first compute dispatch after that point, not the dispatch or its constants. Unexplained; the next
  step is an ETW trace taken in the same run, which needs the trace tools to run without an elevated shell.
  **Cause found (2026-11-10):** an ETW DxgKrnl trace and the Tracy capture of one run, aligned on the 32 ms frame period
  (`tracy_se_20261110_correlated.tracy`, `gpu_dxg.etl`, VR confirmed not running), show the GPU executing almost nothing
  inside the 8.8 ms stall: SE 0.2 ms, every other process together 1.7 ms (VS Code, the Claude app, DWM, bHaptics,
  RustDesk), about 7 ms idle. So it is not GPU contention. It is the solver's commands sitting unsubmitted in the
  driver's command buffer until the game flushes at the end of its CPU frame. One `context->Flush()` at the end of
  `SubmitFrame` removes it (`tracy_se_20261110_flush.tracy`): `SurfacePrep` 9.1 -> 0.09 ms, `RigidSolve` 0.13 ms,
  `RigidSetup` 0.33 ms, `RigidContact` 0.49 ms, a solver total near 1.1 ms against the 3 ms quoted above, while the
  frame stays at 32.6 ms (33.3 ms without the flush). The earlier per-pass figures on SE therefore overstate the
  solver by about 8 ms of misattributed wait, and the frame time was never bound by the solver's GPU work. The flush
  costs about 0.3 ms of CPU in `SubmitFrame`. An interleaved on/off A/B in one SE session (20 characters, 58 claims, four
  12 s windows each, windows over 60 ms or with a compiler running discarded) gave 32.9 ms with the flush and 32.5 ms
  without: no gain, so `FlushAfterSubmit` ships off and is only a switch for GPU timing. The 3 ms and VR figures were
  not remeasured with a flush.
- **VR, 2026-11-09 (null driver, crowd 20 and 30):** `ContactSpheres` 1.1 ms, solver total about 3.1 to 3.3 ms
  (28% of 11.1 ms), against 10 to 11 ms in the VR table above. The cause of the gap was not identified.
- **`clearTestCrowd` crashed SE at the end of both sessions**, in the engine: `BipedAnim::dtor` -> `RemoveAllParts` ->
  `NiAVObject::LookupBoneNodeByName` (`SkyrimSE.exe+0FDDB2D`) on a test character, with no plugin frame in the stack
  (`crash-2026-11-09-17-13-30.log`, `-17-20-03.log`). The results were already written.

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

## Shader compile time versus runtime cost (2026-11-10)

- **`RigidSetup` took 52 s to compile in FXC (67 to 86 s in a parallel run), at any optimisation level.** Nearly all of
  it was the joint's rows living in a `JointRows` struct of six `AxisRows`: each runtime-slot read or write was a six-way
  chain of whole-struct copies that FXC expanded at every call site, and the setup kernel built every axis into it, then
  walked the slots again to finalise and store them.
- **Streaming the rows out fixed both costs.** `BuildAxisRows` builds one axis, and one `[loop]` bounded by the cbuffer
  `HairRigidAxisSlots` (a constant bound lets FXC unroll the builder, which crashes it) finalises each present row and
  stores it straight to `HairRigidRows`, carrying only the row 0 cfm between axes. `JointRows` and its accessors are gone.
  `fxc /T cs_5_0 /O3`: 51.5 s to 2.8 s, 4616 to 3179 instruction slots, 285 to 120 temp registers, and still only the
  one 3-element indexable temp. The GPU-versus-CPU tests pass unchanged.
- **Runtime fell too.** SE, 20 characters, flush on, one session switching shaders with `reloadSolverShaders`, three
  interleaved pairs of 25 s `gpuPassStats` windows: `RigidSetup` 0.37, 0.46 and 0.50 ms with the struct rows against 0.034,
  0.030 and 0.030 ms with the streamed rows. `RigidSolve` (0.63 to 0.86 ms against 0.81 to 0.90) and the contact passes
  moved with run-to-run noise in both arms, so the rows the solver reads are the same.
- **An earlier attempt kept the rows in plain local arrays.** It cut the compile to 16 s and was reverted (`fc6fb7a`): it
  made the kernel slower, 0.76 ms with the struct rows and 1.27 ms with the arrays, because the arrays compiled to four
  runtime-indexed temp arrays of 420 four-component registers each. The streamed version adds no array.

## FXC audit of every kernel (2026-11-11)

`docs/perf-data/fxc_audit.py` compiles each kernel (`cs_5_0`, `/O3`) and reports slots, temp registers, loops and runtime-indexed
temp arrays, to look for another flaw like the `RigidSetup` one. None of the other kernels shows it:

| Kernel                    | Compile s | Slots | Temps | Indexable temps |
| ------------------------- | --------- | ----- | ----- | --------------- |
| `RigidSetup`              | 3.5       | 3179  | 120   | 3 x 4           |
| `RigidSolveIslandColored` | 1.1       | 707   | 42    | 6 x 4           |
| `ContactSpheres`          | 0.1       | 396   | 19    | none            |
| `RigidIntegrate`          | 0.1       | 394   | 15    | none            |
| `RigidFrameBegin`         | 0.1       | 343   | 21    | none            |
| `RigidPredict`            | 0.1       | 271   | 21    | none            |
| `ContactCull`             | 0.1       | 265   | 14    | none            |
| `RigidSolveIsland`        | 0.2       | 214   | 24    | none            |
| `SurfacePose`             | 0.0       | 101   | 9     | 4 x 4           |
| the rest                  | 0.0       | 18-77 | 5-9   | none            |

- Only `RigidSetup` is large, and it is already cheap at run time (0.03 ms). The solve and contact kernels are small, so
  their cost is memory access and divergence, not register pressure or unrolled copies.
- The two runtime-indexed arrays are small: the six colour offsets in the coloured solve and four rows in `SurfacePose`.
