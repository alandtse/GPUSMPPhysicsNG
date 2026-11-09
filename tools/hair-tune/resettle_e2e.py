"""VR strand grab test with injected controllers: no headset and no person needed.

    python tools/hair-tune/grab_e2e.py [--port 8921] [--crowd 2]

Needs a VR game that is already running (the SteamVR null driver is fine) with the devbench server answering, a
save loaded, and you holding the VR lock. The script turns GPU SMP Physics on, spawns frozen test characters, then
injects controller poses and grip state through devbench's vrTrackedSet input and checks the grab path:

  rest, open hands       -> nothing held, HIGGS free on both hands
  hair off the hand      -> the palm points at nothing and HIGGS keeps the hand
  hair in the palm's cast -> the palm points at hair, strands are in reach, HIGGS is kept from that hand only
  grip squeezed          -> the hand closes, strands are held, the controller pulses
  hand moved while held  -> the hair is pulled away from its animation pose and the controller pulses again
  hand pulled far away   -> the strands are dropped and not grabbed again until the grip opens
  grip released          -> nothing held
  hair moved off the hand -> HIGGS given back, then taken from the other hand
  crowd removed          -> HIGGS free on both hands again

It does not check the weapon gate: injected controller state produces no Ready Weapon events in the game.
Exit code 0 when every check passes.
"""

import argparse
import sys
import time

import ingame

REST_LEFT = (-0.2, 1.2, -0.35)
REST_RIGHT = (0.2, 1.2, -0.35)
PULL_RIGHT = (0.27, 1.2, -0.35)  # within the 6 units a strand grabbed at full length can be pulled
FAR_RIGHT = (0.27, 1.2, -0.35)  # the hand stays where it pulled to, near the hair
OPEN_UNTIL_MS = 10000
PULL_START_MS = 20000
PULL_END_MS = 24000
FAR_START_MS = 26000
FAR_END_MS = 29000
HOLD_UNTIL_MS = 34000
END_MS = 60000
SETTLE_SECONDS = 7.0  # for injected hands to reach their poses, and for released hair to stop swinging
MINIMUM_PULLED_DISPLACEMENT = 0.5  # game units the held hair must sit from its animation pose while pulled
NEAR_MISS = 40.0                   # game units between the hair and the hand's line: past grab reach (5) and past the width of a lock of hair
MAXIMUM_PLAYER_DRIFT = 3.0         # game units the player may move while the test holds a character against a hand
MAXIMUM_REST_DISTANCE = 1.0        # game units a held strand may be from the hand when the hand is still


def right_hand(t):
    if t < PULL_START_MS:
        position = REST_RIGHT
    elif t < PULL_END_MS:
        fraction = (t - PULL_START_MS) / (PULL_END_MS - PULL_START_MS)
        position = tuple(a + (b - a) * fraction for a, b in zip(REST_RIGHT, PULL_RIGHT))
    elif t < FAR_START_MS:
        position = PULL_RIGHT
    elif t < FAR_END_MS:
        fraction = (t - FAR_START_MS) / (FAR_END_MS - FAR_START_MS)
        position = tuple(a + (b - a) * fraction for a, b in zip(PULL_RIGHT, FAR_RIGHT))
    else:
        position = FAR_RIGHT
    return position, OPEN_UNTIL_MS <= t < HOLD_UNTIL_MS


def left_hand(t):
    return REST_LEFT, False


def settle_samples(bench, actor, label, seconds=10):
    """Prints, once a second, how far the hair is from where gravity alone would settle it."""
    out = []
    for _ in range(seconds):
        rows = [row for row in bench.query('poseCheck', actorFormID=actor) if row.get('dynamicBones', 0) > 0]
        out.append((round(max((r.get('meanSolvedFromSag', 0.0) for r in rows), default=0.0), 2), round(max((r.get('maxSolvedFromSag', 0.0) for r in rows), default=0.0), 1)))
        time.sleep(1.0)
    print('SETTLE', label, out, flush=True)


def player_position(bench):
    camera = bench.post('camera', {'action': 'get'}, 10)
    return [camera.get('camX', 0.0), camera.get('camY', 0.0), camera.get('camZ', 0.0)]


def hands(bench):
    grip = bench.status().get('grip', {})
    return {hand['hand']: hand for hand in grip.get('hands', [])}, grip


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
        time.sleep(5)
        outfit = bench.query('testCrowdOutfit')
        actor = outfit[0]['actorFormID'] if outfit else 0
        checks.check('test characters stand', bool(outfit), len(outfit))

        # A character placed on the player's hand overlaps the player and would shove it, so the player's collision is off for the run.
        bench.post('console', {'command': 'tcl'}, 10)
        collision_off = True
        started = time.time()
        token = ingame.start_tracked_sequence(bench, ingame.tracked_frames(END_MS, left_hand, right_hand), 'grab-e2e')

        def at(seconds):
            wait = started + seconds - time.time()
            if wait > 0:
                time.sleep(wait)

        at(2.5)
        state, grip = hands(bench)
        weapon = grip.get('weaponDrawn')
        pulses_before = grip.get('hapticPulses', 0)
        checks.check('hands start open and free', not state['right']['closed'] and not state['left']['closed'] and not state['right']['higgsDisabled'] and not state['left']['higgsDisabled'] and state['right']['holding'] == 0,
                     [state['left']['higgsDisabled'], state['right']['higgsDisabled'], state['right']['holding']])

        at(4.5)
        checks.check('hair is placed off the hand', bench.place_hair_at_hand('right', away=NEAR_MISS))
        time.sleep(1.5)
        state, grip = hands(bench)
        checks.check('HIGGS keeps the hand while the palm is not pointed at hair', not state['right']['higgsDisabled'] and state['right']['palmTarget'] == 'none', [state['right']['higgsDisabled'], state['right']['palmTarget']])
        at(7.0)
        checks.check('hair is placed at the right hand', bench.place_hair_at_hand('right'))
        placed = ingame.wait_until(lambda: (lambda s: s['right']['strandsInReach'] > 0 and s['right']['higgsDisabled'])(hands(bench)[0]), 4)
        state, grip = hands(bench)
        checks.check('hair at the right hand is in reach', state['right']['strandsInReach'] > 0, state['right']['strandsInReach'])
        checks.check('HIGGS is kept from the hand over hair', bool(placed), state['right']['higgsDisabled'])
        checks.check('the palm is pointed at hair', state['right']['palmTarget'] == 'hair', state['right']['palmTarget'])
        checks.check('the other hand is left to HIGGS', not state['left']['higgsDisabled'] and state['left']['strandsInReach'] == 0, [state['left']['higgsDisabled'], state['left']['strandsInReach']])

        before = player_position(bench)
        at(OPEN_UNTIL_MS / 1000 + 1.0)
        closed = ingame.wait_until(lambda: (lambda s: s['right']['closed'] and s['right']['holding'] > 0)(hands(bench)[0]), 5)
        state, grip = hands(bench)
        checks.check('the squeezed grip closes the hand and holds strands', bool(closed), [state['right']['closed'], state['right']['holding']])
        drift = max(abs(a - b) for a, b in zip(before, player_position(bench)))
        checks.check('placing hair at a hand does not shove the player', drift <= MAXIMUM_PLAYER_DRIFT, round(drift, 2))
        pulses_at_grab = grip.get('hapticPulses', 0)
        checks.check('the controller pulses on the grab', pulses_at_grab > pulses_before, [pulses_before, pulses_at_grab])
        held = bench.status().get('heldStrands') or []
        distances = [entry.get('distanceToTarget', 1e9) for entry in held]
        checks.check('held strands sit on the hand', bool(distances) and max(distances) <= MAXIMUM_REST_DISTANCE, [round(d, 2) for d in distances])
        checks.check('the weapon is left alone', grip.get('weaponDrawn') == weapon, grip.get('weaponDrawn'))

        at(FAR_START_MS / 1000 - 0.5)
        state, grip = hands(bench)
        checks.check('the grip holds while the hand moves', state['right']['closed'] and state['right']['holding'] > 0, [state['right']['closed'], state['right']['holding']])
        checks.check('a taut strand pulses the controller again', grip.get('hapticPulses', 0) > pulses_at_grab, [pulses_at_grab, grip.get('hapticPulses')])
        pose = [row for row in bench.query('poseCheck', actorFormID=actor) if row.get('dynamicBones', 0) > 0]
        displaced = max((row['maxSolvedFromRest'] for row in pose), default=0.0)
        checks.check('the pulled hair has left its animation pose', displaced >= MINIMUM_PULLED_DISPLACEMENT, round(displaced, 2))

        at(FAR_END_MS / 1000 + 2.0)
        state, grip = hands(bench)
        checks.check('a strand the hand has pulled out of reach is dropped', state['right']['holding'] == 0 and state['right']['dropped'], [state['right']['holding'], state['right']['dropped']])
        checks.check('the grip stays closed and does not grab again', state['right']['closed'], state['right']['closed'])

        at(HOLD_UNTIL_MS / 1000 + 0.3)
        settle_samples(bench, actor, 'after the grip opens, hand left next to the hair', 10)
        claim = next((c for c in bench.status().get('claims', []) if c.get('actorFormID') == actor), {})
        print('EXTERNAL', claim.get('externalColliders'), [[round(v, 1) for v in row[:7]] for row in (claim.get('externalCapsules') or [])][:8], flush=True)
        at(HOLD_UNTIL_MS / 1000 + 12.0)
        state, grip = hands(bench)
        checks.check('the released grip lets go and clears the drop', not state['right']['closed'] and state['right']['holding'] == 0 and not state['right']['dropped'], [state['right']['closed'], state['right']['holding'], state['right']['dropped']])

        at(HOLD_UNTIL_MS / 1000 + SETTLE_SECONDS)
        bench.place_hair_at_hand('left')
        moved = ingame.wait_until(lambda: (lambda s: s['left']['higgsDisabled'] and not s['right']['higgsDisabled'])(hands(bench)[0]), 4)
        state, grip = hands(bench)
        checks.check('HIGGS is handed back when the hair leaves, and taken from the hand it moved to', bool(moved), [state['left']['higgsDisabled'], state['right']['higgsDisabled']])
    finally:
        if collision_off:
            bench.post('console', {'command': 'tcl'}, 10)
        if token:
            ingame.stop_tracked_sequence(bench, token)
        bench.command('clearTestCrowd')

    time.sleep(4)
    state, grip = hands(bench)
    checks.check('HIGGS is free on both hands once the crowd is gone', not state['left']['higgsDisabled'] and not state['right']['higgsDisabled'], [state['left']['higgsDisabled'], state['right']['higgsDisabled']])
    return checks.finish()


if __name__ == '__main__':
    sys.exit(main())
