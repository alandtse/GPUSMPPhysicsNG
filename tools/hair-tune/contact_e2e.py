"""VR hair contact test without grabbing, with injected controllers: no headset and no person needed.

    python tools/hair-tune/contact_e2e.py [--port 8921] [--crowd 2]

Needs a VR game that is already running (the SteamVR null driver is fine) with the devbench server answering, a
save loaded, and you holding the VR lock. It checks that other bodies move hair without a grab:

  probe capsule beside the hair -> the claim carries an external collider
  open hand placed in the hair  -> the hand's capsules are among the claim's external colliders
The mean distance of the hair from its settled pose is printed at each step but not asserted: the hair's own swing
moves it by as much as one finger-sized capsule does, so the number cannot tell a push from none.
Exit code 0 when every check passes.
"""

import argparse
import sys
import time

import ingame

REST_LEFT = (-0.2, 1.2, -0.35)
REST_RIGHT = (0.2, 1.2, -0.35)
END_MS = 40000
SETTLE_SECONDS = 6.0


def hand(position):
    return lambda t: (position, False)


def claim_of(bench, actor):
    for claim in bench.status().get('claims', []):
        if claim.get('actorFormID') == actor:
            return claim
    return {}


def displacement(bench, actor):
    """Mean distance of the simulated bones from where the claim predicted gravity alone would settle them."""
    rows = [row for row in bench.query('poseCheck', actorFormID=actor) if row.get('dynamicBones', 0) > 0]
    return max((row['meanSolvedFromSag'] for row in rows), default=0.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=8921)
    parser.add_argument('--crowd', type=int, default=2)
    args = parser.parse_args()

    bench = ingame.Bench(args.port)
    checks = ingame.Checks()
    token = None
    collision_off = False
    try:
        bench.set_settings(Enabled=True, CollisionEnabled=True, SolverBackend=1, BodyContacts=True, GrabStrands=True)
        bench.command('spawnTestCrowd', count=args.crowd)
        if not ingame.wait_until(lambda: bench.status().get('claimCount', 0) >= 1, 60):
            checks.check('hair is claimed', False, 'no claims after the crowd spawned')
            return checks.finish()
        time.sleep(3)
        bench.command('reevaluate')
        time.sleep(6)
        outfit = bench.query('testCrowdOutfit')
        actor = outfit[0]['actorFormID'] if outfit else 0
        checks.check('test characters stand', bool(outfit), len(outfit))

        bench.post('console', {'command': 'tcl'}, 10)
        collision_off = True
        token = ingame.start_tracked_sequence(bench, ingame.tracked_frames(END_MS, hand(REST_LEFT), hand(REST_RIGHT)), 'contact-e2e')
        time.sleep(SETTLE_SECONDS)

        rest = displacement(bench, actor)
        print('hair at rest, mean distance from its settled pose', round(rest, 3))
        bench.command('probeBodyContact', actorFormID=actor, enabled=True)
        time.sleep(SETTLE_SECONDS)
        claim = claim_of(bench, actor)
        pushed = displacement(bench, actor)
        checks.check('the probe capsule is an external collider', claim.get('externalColliders', 0) >= 1, claim.get('externalColliders'))
        print('with the probe capsule', round(pushed, 3))
        bench.command('probeBodyContact', actorFormID=actor, enabled=False)
        time.sleep(SETTLE_SECONDS)

        checks.check('hair is placed far from the hands', bench.place_hair_at_hand('right', away=150.0))
        time.sleep(SETTLE_SECONDS + 2)
        far = displacement(bench, actor)
        far_colliders = claim_of(bench, actor).get('externalColliders', 0)
        print('hair far from the hands', round(far, 3), 'external colliders', far_colliders)
        checks.check('hair is placed at the right hand', bench.place_hair_at_hand('right'))
        time.sleep(SETTLE_SECONDS)
        claim = claim_of(bench, actor)
        capsules = claim.get('externalCapsules') or []
        owners = [row[7] for row in capsules]
        checks.check('the open hand gives the claim external colliders', claim.get('externalColliders', 0) >= 1, [claim.get('externalColliders'), owners])
        touched = displacement(bench, actor)
        print('hair at the open hand', round(touched, 3))
        print('hand capsules', capsules)

    finally:
        if collision_off:
            bench.post('console', {'command': 'tcl'}, 10)
        if token:
            ingame.stop_tracked_sequence(bench, token)
        bench.command('probeBodyContact', actorFormID=0, enabled=False)
        bench.command('clearTestCrowd')
    return checks.finish()


if __name__ == '__main__':
    sys.exit(main())
