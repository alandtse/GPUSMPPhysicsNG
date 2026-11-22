# Known differences from hdtSMP64

Checked with the suites in [tools/hair-tune](../tools/hair-tune) on SkyrimVR with the null driver. The grab, contact and
outfit suites pass except the checks below.

- **Cape under the rigid solver** moves 0.78 to 1.03 times as much as hdtSMP64 where the parity baseline allows 0.24 to
  0.74. `smp_parity_baseline.json` is stale: run `smp_parity_e2e.py --update-baseline` on a trusted scene.
- **`holdStrand` leaves the pose unchanged** in the `pose_e2e.py` pull check (the solved-from-rest distance stays near
  0.01). A real hand grab pulls hair 8.9 units (`grab_e2e.py`).
- **Skirt rigs whose per-vertex shapes are collision for others** (DD Elanor Timeless Elegance) collide with the rig's own
  tagged shapes, as hdtSMP64 does. On SE the 64 skirt bones sit a mean 0.57 units (max 5.11) from hdtSMP64's, down from 2.05
  (max 9.25) when the claim had no collision at all.
- **Dresses whose per-triangle shape is skinned to simulated bones** (DD Tara dress) collide that shape with the rig's own body
  proxy (`Colision`), as hdtSMP64 does. At rest on SE the 318 bones sit a mean 0.92 units (max 6.4) from hdtSMP64's, the hem
  strands match hdtSMP64's pose in 12 of 12 re-equips (hem strand A 24.0 and L 18.5 in the pelvis frame, hdtSMP64 24.0 and
  18.7).
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
