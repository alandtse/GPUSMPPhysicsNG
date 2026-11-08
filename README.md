# GPUSMPPhysicsNG

GPU hair and cloth physics for Skyrim SE, AE and VR, built as a companion to
[hdtSMP64](https://github.com/alandtse/hdtSMP64). It reads the same SMP XML rigs, steps them with a
rigid-body solver on the GPU that reproduces hdtSMP64's constraints, and draws the skinned meshes from
the solver's result. One DLL covers all three runtimes.

It claims only the meshes whose physics file describes a bone chain; hdtSMP64 keeps simulating
everything else, so the two run side by side.

## Requirements

- hdtSMP64 with plugin interface 2.1 or newer, loaded before this plugin
- [SKSE Menu Framework](https://github.com/alandtse/SKSE-Menu-Framework-3) for the in-game settings page (optional)
- [Community Shaders / Open Shaders](https://github.com/alandtse/open-shaders) for wind (optional); without it the weather's wind is used

## Install

Copy `SKSE/Plugins` into `Data`. The plugin compiles its HLSL on your machine from
`Data/SKSE/Plugins/GPUSMPPhysicsNG/Shaders` the first time a mesh is claimed.

Settings live in `Data/SKSE/Plugins/GPUSMPPhysicsNG/Settings.json`, written with defaults on first run, and on the
"GPU SMP Physics NG" page of SKSE Menu Framework. The log is `GPUSMPPhysicsNG.log` next to the other SKSE logs.

## Open Shaders' built-in GPU SMP Physics

Open Shaders ships the same feature. Run only one claimer: turn off Open Shaders' "GPU SMP Physics"
(its `SmpPhysics.ini`, or its Enabled switch) when you use this plugin.

## Build and test

Needs xmake and MSVC (run from PowerShell; xmake picks MinGW under Git Bash).

```
git submodule update --init --recursive
xmake f -m releasedbg --toolchain=msvc -y
xmake build GPUSMPPhysicsNG
xmake build GPUSMPPhysicsNG-Tests
xmake run GPUSMPPhysicsNG-Tests
```

`GPUSMP_DEPLOY=1` with `SkyrimPluginTargets` or `SkyrimVRPluginTargets` (a `Data` folder) copies the
DLL and shaders into a game install after a build.

## Layout

- `src/smp`: engine-free solver core (rig building, packing, collision, skinning math)
- `src/d3d`: D3D11 solver dispatch; `src/engine`, `src/Features`, `src/plugin`: game integration
- `src/ui`: SKSE Menu Framework settings page
- `shaders`: HLSL
- `tests`: Catch2 suite, including a WARP-device smoke test of the GPU solver

## License

GPL-3.0-or-later with the modding exception in `EXCEPTIONS.md`.
