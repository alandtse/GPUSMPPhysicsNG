# AGENTS.md

Guidance for coding agents working in this repository. Claude Code loads it through `.claude/CLAUDE.md`.

GPUSMPPhysicsNG is a standalone SKSE plugin (one DLL for SE, AE and VR) that runs hdtSMP64's skinned-mesh physics on
the GPU. It started as a feature inside Open Shaders; that branch (`claude/hair-body-capsules`, `1a993b31f5`) is
reference only and nothing is done there any more. Open Shaders is no longer a dependency; it is an optional source of
wind only.

## Build and test

```powershell
xmake f -m releasedbg --toolchain=msvc -y
xmake build
xmake run GPUSMPPhysicsNG-Tests
```

Run xmake from PowerShell, not Bash. All Catch2 cases must pass before a commit.

Deploy to a game's `Data` folder: set `GPUSMP_DEPLOY=1` plus `SkyrimVRPluginTargets` or `SkyrimPluginTargets`, then build.
It copies the DLL (skipped while the game runs) and `shaders/` to `Data/SKSE/Plugins/GPUSMPPhysicsNG/Shaders`.
Deploy only while holding that game's lock (below).

## Layout

- `src/` plugin code; `src/Features/SmpPhysics.cpp` is the feature, `src/d3d/GpuSolver.cpp` the solver host,
  `src/GpuPass.h` + `src/GpuPassTimer.h` per-pass GPU timing (`CS_GPU_PASS`).
- `shaders/` HLSL, shipped as loose files next to the DLL.
- `tests/` Catch2. Test the real code, never a pasted copy of it.
- `tools/hair-tune/` in-game suites driven over devbench; `tools/game/` launcher and quick checks.
- `docs/PARITY.md`, `docs/PERFORMANCE.md`, `docs/perf-data/` measured evidence; update them when a result changes.

## Code rules

- Comments: none by default; one or two lines only when the WHY is not obvious and a change would break it silently.
- Minimal churn: touch only the lines the change needs; no reformatting or reordering.
- Name every D3D11 resource (`Util::SetResourceName`-style names, `"SmpPhysics::Thing"`, views with ` SRV`/` UAV`).
- Wrap each render pass entry in `CS_GPU_PASS("Feature::Pass")`.
- Features disable cleanly on shader-compile or D3D failure; never crash the game.
- New C++ logic gets a Catch2 test. Python is for the in-game tooling only.
- Commits: Conventional Commits `type(scope): description`, title <= 50 chars, body wrapped at 72. Pick the type for
  release impact (build/test/ci are not `fix:`). End with the attribution lines the session gives.
- Local repo with no remote. Do not create or push to one without asking.

## In-game testing

Devbench is the test interface. The plugin owns the `gpusmp.actions` tool (owners `SmpPhysics`, `TestCrowd`); set
`GPUSMP_STANDALONE=1` so `tools/hair-tune/ingame.py` routes there. Ports: SE 8920, VR 8921, but confirm with
`inspect{kind:state}` (the `vr` field tells SE from VR); see `~/.claude/devbench-usage.md`.

Coordination with other agents is mandatory (`~/.claude/skyrim-game-coordination.md`):

- SE and VR are separate games. Take the lock with `~/.claude/tools/skyrim-game.ps1` (acquire, renew, release,
  clear-running) before deploying or launching; never preempt or kill a game another agent holds.
- Back up the target game's DLLs before deploying to one shared with others, and gate the deploy on a successful acquire.
- Launch with `python tools/game/launch.py <SE|VR>` (holder from `GSP_HOLDER`, default `gpusmpng-vr`). It runs
  `backgroundCompile` first and loads the test save. VR uses the SteamVR null driver (`~/vr-null-toggle.ps1`).
- Quit with the `qqq` console command, never taskkill. Release the lock and turn the null driver off afterwards.
- Perf runs mark the machine do-not-disturb; keep every run under 5 minutes.
- An OS modal blocks SE captures; test and screenshot on VR.

Suites (all need a game running with the plugin and `GPUSMP_STANDALONE=1`):

| Script | Covers |
|---|---|
| `tools/hair-tune/grab_e2e.py` | hand grab, HIGGS gate, hold, pull, drop, haptics |
| `tools/hair-tune/contact_e2e.py` | probe and hand capsules as colliders |
| `tools/hair-tune/pose_e2e.py` | outfits, idles, overlay, pull |
| `tools/hair-tune/smp_parity_e2e.py` | comparison against hdtSMP64 (needs its baseline) |
| `tools/hair-tune/limit_probe.py` | measures joint angles against their limits through a hold and release on real hair |
| `tools/hair-tune/wind_probe.py` | measures how far the wind moves real hair, in the head frame, per wind strength |
| `tools/game/check_actions.py`, `check_overlay.py` | quick devbench and overlay sanity (`GSP_PORT`) |

Known failures and their causes are in `docs/PARITY.md`. Clear the test crowd, set the cloak index to `-1` (0 is a real
cloak) and toggle `Enabled` between suites.

## Performance comparisons

`docs/perf-data/smp_ab.py` runs hdtSMP64, a reference build and this plugin on one scene. Lift every throttle so no arm
simulates fewer actors: hdtSMP64 `configs.json` (`autoAdjustMaxSkeletons` false, `maximumActiveSkeletons` 1000,
`budgetMs` 1000, `minCullingDistance` 100000, `minScreenSizePercent` 0, restore the file afterwards) and here
`MaxClaims`/`MaxSimulatedClaims` 256, `SimulationDistance` 20000, `MinScreenSizePercent` 0. The VR null driver caps the
frame near 31 ms and hides frame time, so use SE for frame time and VR for per-pass GPU cost (`gpuPassTiming` command,
`gpuPassStats` query). State the result against the frame budget (VR 11.1 ms, flat 16.7 ms).

## Planned work

- The chain solver stepping path and its leftover names are gone. Wind (drag, cap and gust law from hdtSMP64) and grab
  pins now act on the rigid bodies: wind adds to each body's frame force in `RigidFrameBegin`, a pin is a capped spring
  in `RigidPredict`; a hold frees the angular limits of the joints up to the root. Measured on VR with
  `tools/hair-tune/limit_probe.py` and `wind_probe.py`: a hold takes real joints about 2 rad past their limits and they
  settle back within 0.5 s of the release with or without a return-rate clamp (about 10 rad/s either way), so there is
  no clamp. Freeing the limits moved held strands from 0.02-0.13 to 0.08-0.32 units off the hand (grab_e2e limit 1.0).
  Wind at strength 60 shifts a claim's hair 0.6 units on average (max 4.6) in the head frame; at the default 2 the
  shift (0.04) is the same as the 0.04 noise between two no-wind runs, with that claim 77% sheltered. As a bounds check
  against hdtSMP64 (its wind is known to be over-strong, so no parity is expected), `wind_probe.py --hdt` across two
  launches with hdtSMP64's wind on and off: hdtSMP64 at its default strength 2 moves the same hair 0.40 units on
  average (max 2.0), where this plugin's no-wind runs differ by 0.12 between launches. So the default here is far below
  hdtSMP64's. The wind field is the sky's normalised speed (0 to 1, gusts to 2) and the hair vector is field x 300 x
  strength, so strength 2 at full sky wind is 600 units/s. In a forced SkyrimStormRain the field only averaged 0.22
  (peak 0.39), about 130 units/s at strength 2, which moves hair by 0.03. A 600 units/s vector (strength 9 in that
  storm) shifts it 0.34 on average (max 2.2), about hdtSMP64's 0.40 at its default. Trees bend 3% of height and grass
  13 degrees per unit of field at full wind, for comparison. Still to do: decide whether the default `WindStrength`
  should rise from 2 toward 4 to 6 so a storm shows; the chain-only packing
  (`PackChainTies`, the tie fields, `windChainLoad`, `subtreeLoading`) is kept for now. Re-baseline
  `smp_parity_baseline.json` and rerun the SE frame-time comparison once SE is free.
- Not covered by any suite: Menu Framework page rendering, the Ready Weapon gate, haptic feel, real controller mappings.
- The 31 ms convergence at 30 characters on SE is unexplained.
