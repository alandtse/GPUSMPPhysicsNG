# AGENTS.md

Guidance for coding agents working in this repository. Claude Code loads it through `.claude/CLAUDE.md`.

GPUSMPPhysicsNG is a standalone SKSE plugin (one DLL for SE, AE and VR) that runs hdtSMP64's skinned-mesh physics on
the GPU. Open Shaders is not a dependency; it is an optional source of wind only.

## Build and test

```powershell
xmake f -m releasedbg --toolchain=msvc -y
xmake build
xmake run GPUSMPPhysicsNG-Tests
```

Run xmake from PowerShell, not Bash. All Catch2 cases must pass before a commit.

`tools/game/build_shader_cache.ps1` fills a `ShaderCache` folder from the unit tests (the `RigidSetup` compile, 3 s, is the longest); ship it as `Data/SKSE/Plugins/GPUSMPPhysicsNG/ShaderCache`. Entries are keyed by shader path under the shader
root and checked by a content hash of the source and its includes, so a cache built elsewhere stays valid. The skinning and
debug-line shaders are not in it; they compile quickly.

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
- SE can show a warning modal that blocks devbench captures until it is dismissed. Dismiss it and carry on; do not switch to VR
  to avoid it. When SE captures look wrong, check for a modal first.

Suites (all need a game running with the plugin and `GPUSMP_STANDALONE=1`):

| Script                                            | Covers                                                                             |
| ------------------------------------------------- | ---------------------------------------------------------------------------------- |
| `tools/hair-tune/grab_e2e.py`                     | hand grab, HIGGS gate, hold, pull, drop, haptics                                   |
| `tools/hair-tune/contact_e2e.py`                  | probe and hand capsules as colliders                                               |
| `tools/hair-tune/pose_e2e.py`                     | outfits, idles, overlay, pull                                                      |
| `tools/hair-tune/smp_parity_e2e.py`               | comparison against hdtSMP64 (needs its baseline)                                   |
| `tools/hair-tune/limit_probe.py`                  | measures joint angles against their limits through a hold and release on real hair |
| `tools/hair-tune/wind_probe.py`                   | measures how far the wind moves real hair, in the head frame, per wind strength    |
| `tools/game/check_actions.py`, `check_overlay.py` | quick devbench and overlay sanity (`GSP_PORT`)                                     |

Known failures and their causes are in `docs/PARITY.md`. Clear the test crowd, set the cloak index to `-1` (0 is a real
cloak) and toggle `Enabled` between suites.

## Performance comparisons

`docs/perf-data/smp_ab.py` runs hdtSMP64, a reference build and this plugin on one scene. Lift every throttle so no arm
simulates fewer actors: hdtSMP64 `configs.json` (`autoAdjustMaxSkeletons` false, `maximumActiveSkeletons` 1000,
`budgetMs` 1000, `minCullingDistance` 100000, `minScreenSizePercent` 0, restore the file afterwards) and here
`MaxClaims`/`MaxSimulatedClaims` 256, `SimulationDistance` 20000, `MinScreenSizePercent` 0. The VR null driver caps the
frame near 31 ms and hides frame time, so use SE for frame time and VR for per-pass GPU cost (`gpuPassTiming` command,
`gpuPassStats` query). State the result against the frame budget (VR 11.1 ms, flat 16.7 ms).

## Open work

- Re-baseline `smp_parity_baseline.json` (`smp_parity_e2e.py --update-baseline`).
- Not covered by any suite: Menu Framework page rendering, the Ready Weapon gate, haptic feel, real controller mappings.
- `WindStrength` defaults to 6 (hair moves about as far as under hdtSMP64's default in a storm); one strength serves hair
  and cloth, each bone's drag capped at 400 units/s^2.
