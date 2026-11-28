# Known differences from hdtSMP64

Checked with the suites in [tools/hair-tune](../tools/hair-tune) on SkyrimVR with the null driver. The grab, contact and
outfit suites pass except the checks below.

- **Cape under the rigid solver** moves 0.16 to 0.19 times as much as hdtSMP64 on VR where the parity baseline allows 0.24
  to 0.74 (it has also measured 0.78 to 1.03); the other eight checks of `smp_parity_e2e.py` pass there, the wig 0.71 units
  from hdtSMP64's and the cape 1.97. Carrying bone scale into the collision skinning left these as they were (0.73, 1.91 and
  0.19 before it). `smp_parity_baseline.json` is stale: run `smp_parity_e2e.py --update-baseline` on a trusted scene. On SE
  the suite does not give a result from an outdoor Whiterun cell: hdtSMP64's wig does not move in its arm.
- **`holdStrand` leaves the pose unchanged** in the `pose_e2e.py` pull check (the solved-from-rest distance stays near
  0.01). A real hand grab pulls hair 8.9 units (`grab_e2e.py`).
- **Skirt rigs whose per-vertex shapes are collision for others** (DD Elanor Timeless Elegance) collide with the rig's own
  tagged shapes, as hdtSMP64 does. On SE the 64 skirt bones sit a mean 0.57 units (max 5.11) from hdtSMP64's, down from 2.05
  (max 9.25) when the claim had no collision at all.
- **Dresses whose per-triangle shape is skinned to simulated bones** (DD Tara dress) collide that shape with the rig's own body
  proxy (`Colision`), as hdtSMP64 does. At rest on SE the 318 bones sit a mean 0.92 units (max 6.4) from hdtSMP64's, the hem
  strands match hdtSMP64's pose in 12 of 12 re-equips (hem strand A 24.0 and L 18.5 in the pelvis frame, hdtSMP64 24.0 and
  18.7).
  Standing, its skirt bones sit within 0.03 units of hdtSMP64's on average along the radius from the pelvis (worst bone 0.6),
  since the collision skinning carries the bones' world scale: without it the body proxy of an actor of scale 1.03 was up
  to a unit too small and the skirt rested 0.6 to 1.1 units closer to the body. Running onto the Dragonsreach stairs no
  frame shows skin in 6 runs, as under hdtSMP64; with both capped to 66 fps, so that a frame takes two solver steps, none
  in 6 runs against small openings in 3 of hdtSMP64's 6. At the start of a sprint both show the same two openings over the
  buttocks: with both capped to 100 fps, 2760 px.ms of skin in the first 0.4 s against hdtSMP64's 2700 (four sets each,
  sets from 1250 to 4530), and none after that against 60. Uncapped the plugin runs faster than hdtSMP64 (7.9 against
  9.0 ms a frame) and shows more there (3800 against 1900), so compare the two at one frame rate.
- **Skirts under fast movement** pose the collision surface at the pose the bodies began the step from, blended to the
  frame's pose at each substep, as hdtSMP64 steps its bodies and collision on one clock. Running at 360 units/s, the
  spheres behind a face on DD Elanor Timeless Elegance fell from 166 to about 40 on average and no skin shows through the
  skirt in a full run. Each contact takes the velocity of the driver carrying the face or collider it touches, not the
  pelvis's: in a real sprint, spheres more than 3 units behind a face fell from a mean of 3-4 (max 21-30) to 0, and the
  worst mean strand offset from hdtSMP64 from about 24 to about 12.5 units.
- **hdtSMP64's own reference is noisy** between runs, so small distance checks can flip.
- **Wind** is not hdtSMP64's scale (its wind is over-strong). The default `WindStrength` is chosen so hair moves about as
  far as under hdtSMP64's default in a storm.
- **Not covered by any suite:** the settings page, the Ready Weapon gate, haptic feel, real controller mappings.

Before running `pose_e2e.py`, clear the test crowd, set the cloak index to `-1` (0 is a real cloak) and toggle `Enabled`.

## The renderer's context lock

The plugin records on the game's immediate `ID3D11DeviceContext`, which Microsoft documents as single-threaded. Mods that
submit from their own workers - RaceMenu, CommunityShaders' flowmap, and MuDynamicNormalMap, whose TBB workers update
normal maps on the main GPU - take the engine's own renderer lock (`RenderData::lock`) around their calls; this plugin used
to record without it. Two threads in that context corrupt the driver's internal bookkeeping, which then trips on a later
call from either of them: the crash lands inside `nvwgf2umx.dll` with no plugin frame on the stack, so it reads as a
driver fault rather than a race. MuDynamicNormalMap made it reproducible on an RTX 5090 pair (`GPUDeviceIndex = -1` finds
no second GPU there and falls back to the main one); with its GPU path off the same scene does not crash.

Every entry point that touches the context now takes the same lock, and takes it by `TryEnterCriticalSection` rather than
waiting, so the plugin can never hold up a frame:

| Entry point               | On a busy lock                                                              |
| ------------------------- | --------------------------------------------------------------------------- |
| `GpuSolver::SubmitFrame`  | the frame is not recorded, so every claim keeps the engine's pose           |
| `GpuSolver::Poll`         | the readbacks stay pending and are resolved by the next admitted frame      |
| `GpuSolver::AddClaim`     | the claim is declined for this frame only and is registered on the next one |
| `GpuSkinning::BuildBatch` | no palette is rebuilt, and `Apply` falls back to the engine's matrices      |
| `GpuSkinning::Apply`      | the mesh is drawn from the engine's matrices                                |
| `DebugLines::Render`      | the overlay skips the frame                                                 |
| `GpuPassTimer::NextFrame` | the open timing frame stays open and is closed by the next admitted frame   |

Windows critical sections are recursive per owning thread, so the two entry points that run inside the engine's world
render (`DebugLines::Render` from `OnDeferredGeometryEnd`, `GpuSkinning::Apply` from the bone-upload hook) nest inside a
lock the render thread already holds instead of being refused. `GpuPassTimer` is not measured on such a frame while a
submission is refused, so the reported per-pass cost covers the frames that recorded.

A refusal of `AddClaim` is marked retryable and does not set the claim's `gpuRegistrationFailed`, which is what sends a
claim back to hdtSMP64 for good; the claim is registered on the next frame instead.

The counters are in the `status` query under `rendererLock`: `installed`, `acquired`, `busy` and `unlocked`. A non-zero
`busy` means another mod was submitting on the context during a frame this plugin would have recorded; `unlocked` means
the plugin never reached `kDataLoaded` and is recording without a lock.
