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
  proxy (`Colision`), as hdtSMP64 does. At rest on SE the 318 bones sit a mean 0.86 units (max 11.2) from hdtSMP64's, and the
  hem strands 1 to 4 units where they sat 7 to 10 units too close to the body. Strands A, G, H and L still hang 4 to 9
  units behind hdtSMP64's, and the thigh shows through the front of the skirt. Their spheres start 4 to 5 units inside the thigh
  shell and the one-sided contact does not lift them out; the rest-pose retarget never runs for this rig.
- **hdtSMP64's own reference is noisy** between runs, so small distance checks can flip.
- **Wind** is not hdtSMP64's scale (its wind is over-strong). The default `WindStrength` is chosen so hair moves about as
  far as under hdtSMP64's default in a storm.
- **Not covered by any suite:** the settings page, the Ready Weapon gate, haptic feel, real controller mappings.

Before running `pose_e2e.py`, clear the test crowd, set the cloak index to `-1` (0 is a real cloak) and toggle `Enabled`.
