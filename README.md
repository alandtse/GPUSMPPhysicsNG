# GPUSMPPhysicsNG

Standalone GPU SMP physics for Skyrim SE/AE/VR, built as a companion to hdtSMP64.
It solves hair and cloth rigs on the GPU from the same SMP XML rigs hdtSMP64 loads.

Status: scaffold. The engine-free solver core (`src/smp`), its HLSL (`shaders`) and its
Catch2 suite (`tests`) build and pass on their own. The D3D11 layer and the SKSE plugin
glue are being ported from Open Shaders' GPU SMP Physics feature.

## Build and test

```
xmake f -m releasedbg --toolchain=msvc -y
xmake build GPUSMPPhysicsNG-Tests
xmake run GPUSMPPhysicsNG-Tests
```

## License

GPL-3.0-or-later with the modding exception in `EXCEPTIONS.md`.
