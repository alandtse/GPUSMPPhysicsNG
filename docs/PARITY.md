# Parity with the in-tree Open Shaders feature

Verified 2026-11-08 on SkyrimVR (null driver, injected controllers) with the suites in
[tools/hair-tune](../tools/hair-tune). The Open Shaders side is the pre-port build (branch
`claude/hair-body-capsules` at `1a993b31f5`), run in the same session as this plugin.

| Suite | This plugin | Open Shaders in-tree |
|---|---|---|
| `grab_e2e.py` (hand grab, HIGGS gate, hold, pull, drop, haptics) | 22 of 22 | 22 of 22 on 2026-10-21 |
| `contact_e2e.py` (probe and hand capsules as external colliders) | 5 of 5 | not rerun |
| `pose_e2e.py` (outfits, idles, overlay, pull) | 12 of 13 | pull check also fails (below) |
| `smp_parity_e2e.py` (against hdtSMP64) | 11 of 14 | 11 of 14, same checks |

## Known failures, identical in both builds

These are not port gaps; the in-tree feature shows the same numbers.

- **Chain solver does not move the hair.** `smp_parity_e2e.py` measured a spread ratio of 0.00 for the wig and the
  cape under the chain solver, against a baseline that allowed 0.45 to 1.25 and 0.05 to 0.29. The chain solver has
  since been removed, and so have its parity legs and baseline entries; the rigid solver steps every claim.
- **Cape under the rigid solver** moves 0.78-1.03 times as much as hdtSMP64 against an allowed 0.24 to 0.74.
- **`holdStrand` leaves the pose unchanged** (`pose_e2e.py`, pull check): the maximum solved-from-rest distance
  stays at about 0.01 on every character. The real hand grab does pull hair 8.9 units (`grab_e2e.py`).
- `smp_parity_baseline.json` still holds the rigid limits measured before the removal; run `--update-baseline`
  on a trusted scene to re-baseline them.

## Notes on the runs

- Distances vary between runs when hdtSMP64's own reference is noisy: one pre-port run read hdtSMP64 against itself at
  0.46 and the wig at 0.77, the repeat at 0.04 and 2.15; this plugin read 2.20 and 2.14 in two runs.
- The parity test leaves the crowd stripped with a cloak and the claims set up for the parity legs. Clear the crowd, set the
  cloak to `-1` (index 0 is a real cloak) and toggle `Enabled` before running `pose_e2e.py`.
- Not covered by any suite: the settings pages rendering in the Menu Framework, the Ready Weapon gate, haptic feel,
  and real controller mappings.
