# Performance against hdtSMP64

Measured on Skyrim SE 1.7.104 with the test crowd (wigged characters, 10 bases repeated) in one scene. Both plugins were
built with Tracy; zone totals come from `tracy-capture` and `tracy-csvexport`, 20 s per arm. Harnesses are in
[perf-data](perf-data).

## Method

- Every throttle is lifted so both arms simulate the same characters: hdtSMP64 `configs.json` (`autoAdjustMaxSkeletons`
  false, `maximumActiveSkeletons` 1000, `budgetMs` 1000, `minCullingDistance` 100000, `minScreenSizePercent` 0) and here
  `MaxClaims` and `MaxSimulatedClaims` 256, `SimulationDistance` 20000, `MinScreenSizePercent` 0.
- hdtSMP64's `PhysicsStep` with the plugin disabled is compared with the plugin's CPU `Step`, its GPU `RigidContact` and
  `RigidSolve`, and hdtSMP64's `PhysicsStep` on what the plugin does not claim. GPU time is asynchronous, so the sum is a
  conservative total.

## Result, wigs, per frame

![Frame cost, hdtSMP64 alone against hdtSMP64 with this plugin](img/perf-frame-cost.png)

| Crowd | hdtSMP64 | Plugin cpu + gpu + residual | Ratio |
| ----- | -------- | --------------------------- | ----- |
| 5     | 3.4 ms   | 0.76 + 1.42 + 0.14 = 2.3 ms | 0.68  |
| 10    | 4.8 ms   | 1.16 + 1.79 + 0.25 = 3.2 ms | 0.67  |
| 30    | 15.3 ms  | 4.9 ms                      | 0.32  |
| 60    | 32.0 ms  | 3.34 + 5.73 + 2.58 = 11.7   | 0.36  |

At 30 characters hdtSMP64 ranged 15.3 to 20.5 ms over three runs, so its cost is noisier than the plugin's. The Yue
Qingshu Dress crowd of 30 costs hdtSMP64 36.6 ms against the plugin's 10.8 ms (about 0.29), from separate launches.

At default throttles (30 characters, hdtSMP64 simulating about 5 skeletons) hdtSMP64 costs 2.9 ms and the plugin 4.1 ms,
because the plugin simulates all 72 claims. The advantage is per character, not at defaults.

Checked again at 1852d5a, after cloth began to collide triangle against vertex, as frame time with the same crowd and every
throttle lifted (one launch per crowd size, 15 s windows, two per arm):

| Crowd | hdtSMP64 frame | Plugin frame | Plugin cpu + gpu     | Added over no crowd, plugin against hdtSMP64 |
| ----- | -------------- | ------------ | -------------------- | -------------------------------------------- |
| 0     | 8.9 ms         | 7.1 ms       |                      |                                              |
| 10    | 13.6 to 14.1   | 10.1 ms      | 0.32 + 3.58 = 3.9 ms | 3.0 against 5.0 ms (0.59)                    |
| 30    | 30.3 ms        | 14.0 ms      | 0.72 + 4.0 = 4.7 ms  | 6.9 against 21.4 ms (0.32)                   |
| 60    | 52.1 to 53.2   | 19.3 ms      | 1.35 + 5.5 = 6.8 ms  | 12.2 against 43.8 ms (0.28)                  |

The ratios above (0.67, 0.32, 0.36) still hold. At 60 the frame is long enough for two solver steps.

The Yue Qingshu Dress crowd of 30, measured the same way: hdtSMP64 52.5 to 55.4 ms a frame against the plugin's 19.1 ms
(1.5 ms of CPU and 5.1 ms of GPU, two solver steps a frame); added over no crowd 12.0 against about 45 ms, 0.27, where
0.29 was measured at release.

## Where the plugin's time goes

Per call on SE, 10/20/30 characters: `ContactSpheres` 0.64/1.18/1.13 ms and `RigidSolve` 0.74/0.77/0.82 ms. On VR
(null driver) the same passes cost 0.33 to 0.75 ms and about 0.58 ms, so about 2.5 ms of an 11.1 ms frame at 20
characters. Contact is the largest pass.

## Cost of the skirt contact fix

On SE, Tracy, 126 claims (30 characters in DD Elanor Timeless Elegance), every throttle lifted, one run per arm. Posing the
collision surface on the bodies' clock and giving each contact its driver's velocity added 0.87 ms of main thread
(`Step` 3.82 to 4.69 ms, all of it the per-substep contact callback) and 0.34 ms of GPU (`SurfacePrep` now runs per
substep, 0.17 to 0.36 ms; `ContactSpheres` per call is unchanged). Bounding the callback's driver loop to the packed
claims and building the delta once per step takes `Step` to 3.76 ms and the callback from 1.67 to 0.24 ms. Frame
generation and the engine frame lock were not changed, so frame time is not reported.

## Things that did not help

- Routing small islands to the sequential `RigidSolveIsland` kernel made `RigidSolve` slower (0.62 ms coloured-only
  against 1.5 to 2.8 ms).
- Tighter deep-push leaf bounds could save at most about 0.35 ms per frame at 20 characters and cost a new record format.
- Prepacking the coloured solve's body table: its build already costs nothing measurable.
- `FlushAfterSubmit` removes a misattributed 8 to 9 ms wait from the per-pass timers on SE (the commands sit in the
  driver until the game flushes) but gave no frame-time gain, so it ships off. Turn it on only when reading GPU pass times.
- Moving the per-step inputs, driver poses, driver deltas and the rigid frame inputs out of CPU-writable buffers, as was done
  for the static contact lists: 14.8 to 15.0 ms against 14.7 to 14.9 with 30 wigged characters and 19.25 against 19.1 with
  30 in the Yue Qingshu Dress. These hold one record per tree or body, not one per contact thread, so nothing waits on them.

## Contacts posed at the end of the substep

Posing the body surfaces at the end of each substep instead of the start (`SmpStepClock::EndFraction`) adds no pass or
dispatch. SE, Tara dress, 500-frame sprint, frame time with the recorder running: 10.2 to 10.4 ms mean (p99 13.2 to 13.5)
before, 10.0 to 10.3 ms (p99 12.2 to 12.5) after. Deep contacts (>4 units) at the drawn pose fell from 39-40 to 11-20 per
500 frames. `gpuPassStats` was not read for this change.

## Frame time with and without waiting for the solver

The `SameFrameResults` setting measured here has since been removed: once the step followed hdtSMP64's settings and kept the
frame's remainder, a dress drawn without the wait was as clean as one drawn with it, and the wait cost about 7 ms a frame.

`smp_ab.py` on SE, 2026-11-23, with the `SameFrameResults` setting on and off, against hdtSMP64 (mean of two windows).
The budget is 16.7 ms.

| Crowd (claims) | hdtSMP64 | Plugin, default | Plugin, `SameFrameResults` |
| -------------- | -------- | --------------- | -------------------------- |
| 0 (16)         | 19.3 ms  | 17.0 ms         | 19.5 ms                    |
| 10 (46)        | 24.1 ms  | 20.2 ms         | 26.0 ms                    |
| 20 (76)        | 28.7 ms  | 24.5 ms         | 33.9 ms                    |

`SameFrameResults` blocks the main thread for the solver's GPU time, and a frame over 16.7 ms takes two solver steps, so
the wait was 2.2, 5.6 and 8.7 ms. It is off by default: a GPU-skinned mesh is drawn from this frame's solve either way,
because its palette is built from the solver's world buffer right after the step is submitted. Measured as drawn
(`tools/game/patch_probe.py --drawn`, 12 sprints per arm in a physics dress, frames with skin further through the cloth
than hdtSMP64's worst): 1 of 2770 with the wait, 9 of 2872 without and at most two in a row, none for hdtSMP64. Without
`--drawn` the probe measures node poses, which lag one to two frames when the setting is off.

## Sequential depth of the constraint solve

`RigidSolve` runs one thread group per island, so it lasts as long as the island with the most colour classes. The player's
`softbodyFix.xml` ties one body to 35 others, which took 39 classes and 2.66 ms per step for the whole scene. Solving such
a body's joints as one class, each against a share of its mass, takes 15 classes and 1.2 ms per step. Two changes that
kept the sequential order did not help: letting one thread solve a run of connected constraints without a barrier
(3.9 ms), and skipping classes without contacts in the push sweep (2.62 ms).

At 20 characters `ContactSpheres` is the largest pass, about 3 ms per step.

## At hdtSMP64's own step settings

Until 2026-11-24 the solver stepped at 60 Hz with 16 iterations whatever hdtSMP64 was configured to do, so the comparisons
above against an hdtSMP64 at 90 Hz and 32 iterations were not like for like. The solver now reads `min-fps`, `maxSubSteps`,
`numIterations` and `erp` from hdtSMP64's `configs.json` and `userConfigs.json`. With 90 Hz and 32 iterations on both sides, SE,
Dragonsreach, the player in the Tara dress (five physics files) and five other characters in range, nothing else measuring:

| Scene                          | hdtSMP64 frame  | This plugin frame |
| ------------------------------ | --------------- | ----------------- |
| Standing, two runs each        | 18.6, 19.1 ms   | 16.5, 16.7 ms     |
| Running, three runs each       | 13.5 to 14.1 ms | 11.4 to 11.9 ms   |
| Running, same-frame results on | n/a             | 19.7 ms           |

With hdtSMP64's throttles lifted in both of its config files (`userConfigs.json` overrides `configs.json`) and a test crowd, standing,
15 s windows: no crowd 20.3 ms against 18.2 ms; ten extra characters 30.1 ms against 21.7 ms (46 physics files on the GPU).
The same test a day later, after the two solver fixes above and the collision fixes that followed (1852d5a): no crowd 8.9 ms
against 7.1 ms; ten extra characters 13.6 to 14.1 ms against 10.1 ms (47 physics files). The release this branch started
from (f56246c) gave 14.2 ms with ten extra characters in the same hour. Clearing the test crowd can crash the game on any
of these builds, the release included, so a run that ends early has to be repeated.

The rows come from different sessions, so compare within a row. The plugin's solver passes take about 7 ms of GPU time a
frame at these settings (contact passes 2.7, surface posing 2.5, solve 2.0).

The plugin's own `patchProbe` capture costs about 5 ms a frame on this plugin's side only (it reads the drawn palette back),
so a frame time taken while it runs reads this plugin slower than hdtSMP64 when it is not.

## As the game is set up: a regression found and removed

The sections above lift every throttle and run a test crowd. Measured instead as a player has it (hdtSMP64's own
`userConfigs.json`, the plugin's default limits) on one save, Dragonsreach, with the player's dress and hair and three
guards' fur in view (11 claims, 17095 spheres, 10809 cloth vertices, 20254 cloth triangles), the plugin had become five
times slower than hdtSMP64 without any line of this file showing it. `tools/game/perf_check.py` now records every solver
commit in [perf_log.jsonl](perf-data/perf_log.jsonl); the bisect it started with, standing still, frame time in ms:

| Commit                                             | Plugin | hdtSMP64 | GPU per step |
| -------------------------------------------------- | ------ | -------- | ------------ |
| `f56246c`, where the branch starts                 | 9.6    | 9.7      | 1.2          |
| `648468f`, before cloth triangle contacts          | 14.0   | 9.8      | 2.0          |
| `ce84adc`, cloth triangle contacts by default      | 24.6   | 9.2      | 5.1          |
| `94eb254`, after the step rate followed hdtSMP64's | 47.4   | 9.5      | 6.2          |
| `80e4d28`, a bone's triangles folded in chunks     | 17.6   | 9.7      | 4.3          |
| `9316a49`, static lists kept in GPU memory         | 7.8    | 9.0      | 1.9          |

Running, `9316a49` is at 6.5 ms against 7.3.

- The cost was in the cloth contact passes, and most of it was not their arithmetic: the sphere list and the three cloth
  lists sat in CPU-writable buffers that every thread of a pass read each step. With the passes skipped altogether the
  frame was 9.0 to 9.2 ms, so the solve and the surface refit were never the problem here.
- The frame time has several steady states. A frame longer than the 90 Hz tick takes a second solver step, each step
  costs GPU time, and the frame stays long: the same build ran at 13.6, 30 and 47 ms in different sessions. A cost per step
  under about 4 ms keeps this scene at one step.
- The per-pass GPU timers put a pass's cost on the pass that reads its output (posing 10809 vertices was shown as 0.9 ms,
  the merge that waited for it as 2 ms). Their total is right; to cost a pass, skip it and read the frame time.

## Building a claim when an outfit goes on

Measured on SE on 2026-11-25: 20 extra characters in Dragonsreach, all dressed at once in the Qingshu dress (`YQS.xml`),
frames counted over the 25 s that follow. A claim's collision data is built on the CPU when its mesh attaches.

|                                        | Before | After  |
| -------------------------------------- | ------ | ------ |
| `YQS.xml` claim build, average         | 136 ms | 39 ms  |
| Frame rate while the 20 dress, average | 41 fps | 63 fps |
| Slowest quarter second                 | 0 fps  | 4 fps  |

A newly attached mesh is posed before its bones are placed, so some of its triangles stretch 300 to 600 units. The
surface grid entered each triangle in every 3-unit cell its box spans, which took 30 to 135 ms a surface; it now grows its
cells when the entries pass `kMaximumCellEntries`. What is left of the dress's build is the body proxies (14 to 30 ms),
the rest-pose probe (about 17 ms) and the spheres (3 ms); hair files spend theirs on spheres (`vipress.xml`, 36 of 47 ms).
The frame still stalls for a quarter second or more while a crowd dresses, and spawning the 20 undressed characters, with no
dress to claim, drops to 12 to 15 fps in its worst quarter second, so not all of the stall is this plugin's.

## Per-frame bookkeeping on the main thread

With 95 simulated claims the plugin's main-thread step took 4.0 ms a frame (Tracy, 2026-11-25), of which 0.75 ms rebuilt two
tables that only change with the claim set: the set of every simulated node, and the map of which claim last wrote each
node. Both are now rebuilt only when a claim is added, replaced or removed, or when the claims that write differ from the
frame before. SE, 30 extra characters in the Qingshu dress, hdtSMP64's throttles lifted, two readings each, one launch each:

|            | Before        | After         | hdtSMP64    |
| ---------- | ------------- | ------------- | ----------- |
| Frame time | 18.8, 19.0 ms | 17.4, 17.5 ms | 50 to 52 ms |

After that the step was 3.29 ms. Each claim with an external driver (67 of the 95) also copied its external colliders into
the shared collider buffer every frame, one `UpdateSubresource` each; `a486fa7` uploads them only when their bytes change.
Packing the frame's claims fell from 0.58 to 0.16 ms and the step to 2.84 ms. What is left, per frame: recording the
dispatches 0.73 ms, writing the pose to the nodes 0.40, reading the collision drivers 0.31, updating child nodes 0.23,
recording skinning poses 0.22. Frame time was not compared for this one: the machine was busy and both plugins varied 2 to
3 ms between launches.

## A dense robe (issue 52)

Issue 52 measured one character in the DDV Nocturnal Robe (`Noct.xml`, 7,000 to 8,800 spheres against two dense body
meshes) at 3.6 to 4.4 ms per substep in `ContactSpheres` on 1.5.0, about 6.5 ms of frame time. SE, one test character
in the robe, 2026-11-26, one launch each:

|                           | Frame before the robe | Robe on, 20 s | Robe on, 90 s | Contact step, before to with the robe |
| ------------------------- | --------------------- | ------------- | ------------- | ------------------------------------- |
| 1.5.1                     | 10.1 ms               | 11.5 ms       | 9.9 ms        | 2.86 to 3.31 ms                       |
| With the nearer-face rule | 8.6 ms                | 8.2 ms        | 8.1 ms        | 1.35 to 1.31 ms                       |

1.5.1 already searches a surface within its own allowance first and caps the deep search at 256 nodes. A sphere hanging
near the body without touching it still spent that budget every step. It now stops at the first face it is in front of:
such a face ends the deep search, and it also means the sphere is outside the mesh, so a face further off cannot take it.
The two rows are separate launches, so compare each row's own before and after.

## Not measured

VR frame time (the null driver caps the frame near 31 ms), capes, AE, frame generation state.
