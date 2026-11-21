"""VR strand grab test with injected controllers: no headset and no person needed.

    python tools/hair-tune/grab_e2e.py [--port 8921] [--crowd 2]

Needs a VR game that is already running (the SteamVR null driver is fine) with the devbench server answering, a
save loaded, and you holding the VR lock. The script turns GPU SMP Physics on, spawns frozen test characters, then
injects controller poses and grip state through devbench's vrTrackedSet input and checks the grab path:

  rest, open hands       -> nothing held, HIGGS free on both hands
  hair off the hand      -> the palm points at nothing and HIGGS keeps the hand
  hair in the palm's cast -> the palm points at hair, strands are in reach, HIGGS is kept from that hand only
  grip squeezed          -> the hand closes, strands are held, the controller pulses
  hand moved while held  -> the held strands follow the hand, the hair is pulled away from its animation pose and the controller pulses again
  hand moved, no grip    -> the same strands stay where they were
  hand pulled far away   -> the strands are dropped and not grabbed again until the grip opens
  grip released          -> nothing held
  hair moved off the hand -> HIGGS given back, then taken from the other hand
  crowd removed          -> HIGGS free on both hands again

It does not check the weapon gate: injected controller state produces no Ready Weapon events in the game.
Exit code 0 when every check passes.
"""

import argparse
import math
import statistics
import sys
import time
import urllib.error

import ingame

REST_LEFT = (-0.2, 1.2, -0.35)
REST_RIGHT = (0.2, 1.2, -0.35)
PULL_RIGHT = (0.27, 1.2, -0.35)  # within the 6 units a strand grabbed at full length can be pulled
FAR_RIGHT = (1.1, 1.2, -0.35)
OPEN_UNTIL_MS = 10000
PULL_START_MS = 20000
PULL_END_MS = 24000
FAR_START_MS = 26000
FAR_END_MS = 29000
HOLD_UNTIL_MS = 34000
END_MS = 52000
SETTLE_SECONDS = 7.0  # for injected hands to reach their poses, and for released hair to stop swinging
MINIMUM_PULLED_DISPLACEMENT = 0.5  # game units the held hair must sit from its animation pose while pulled
NEAR_MISS = 40.0                   # game units between the hair and the hand's line: past grab reach (5) and past the width of a lock of hair
MAXIMUM_PLAYER_DRIFT = 3.0         # game units the player may move while the test holds a character against a hand
MINIMUM_STRAND_FOLLOW = 1.0  # game units a held strand must move when the hand moves PULL_RIGHT from REST_RIGHT (about 4.9)
MAXIMUM_UNHELD_MOVE = 1.0    # game units the same strands may move when the hand moves the same way with no grip
CONTROL_PULL_START_MS = 8000
CONTROL_PULL_END_MS = 10500
CONTROL_END_MS = 15000
CONTROL_START_ATTEMPTS = 25  # one second apart, for the main sequence's remaining frames to run out
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


def bone_positions(bench, actor, bones):
    """The solver pose of the given rig bones of an actor, in each claim's root frame, keyed by claim actor and bone."""
    positions = {}
    for claim in bench.query('solverBones', actorFormID=actor).get('claims', []):
        for bone in claim.get('bones', []):
            if bone['bone'] in bones:
                positions[(claim['actorFormID'], bone['bone'])] = bone['local']
    return positions


def moved_distances(before, after):
    return sorted(math.dist(after[key], before[key]) for key in before if key in after)


def unheld_strand_move(bench, actor, bones):
    """Moves the right hand as the pull does with the grip never pressed, and returns how far the given bones moved."""
    def right(t):
        position = REST_RIGHT
        if CONTROL_PULL_START_MS <= t < CONTROL_PULL_END_MS:
            fraction = (t - CONTROL_PULL_START_MS) / (CONTROL_PULL_END_MS - CONTROL_PULL_START_MS)
            position = tuple(a + (b - a) * fraction for a, b in zip(REST_RIGHT, PULL_RIGHT))
        elif t >= CONTROL_PULL_END_MS:
            position = PULL_RIGHT
        return position, False

    token = None
    for _ in range(CONTROL_START_ATTEMPTS):
        try:
            token = ingame.start_tracked_sequence(bench, ingame.tracked_frames(CONTROL_END_MS, left_hand, right), 'grab-e2e-control')
            break
        except urllib.error.HTTPError as error:
            if error.code != 409:  # 409: devbench keeps the stopped sequence's owner until its frames run out
                raise
            time.sleep(1.0)
    if token is None:
        return []
    started = time.time()
    try:
        time.sleep(2.0)
        bench.place_hair_at_hand('right')
        time.sleep(max(0.0, started + CONTROL_PULL_START_MS / 1000 - 0.5 - time.time()))
        before = bone_positions(bench, actor, bones)
        time.sleep(max(0.0, started + CONTROL_PULL_END_MS / 1000 + 1.0 - time.time()))
        after = bone_positions(bench, actor, bones)
    finally:
        ingame.stop_tracked_sequence(bench, token)
    return moved_distances(before, after)


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

        at(PULL_START_MS / 1000 - 0.5)
        held_bones = {entry['bone'] for entry in bench.status().get('heldStrands') or []}
        bones_before = bone_positions(bench, actor, held_bones)

        at(FAR_START_MS / 1000 - 0.5)
        followed = moved_distances(bones_before, bone_positions(bench, actor, held_bones))
        follow = statistics.median(followed) if followed else 0.0
        checks.check('the held strands follow the pulling hand', follow >= MINIMUM_STRAND_FOLLOW, [round(f, 2) for f in followed])
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

        at(HOLD_UNTIL_MS / 1000 + 2.0)
        state, grip = hands(bench)
        checks.check('the released grip lets go and clears the drop', not state['right']['closed'] and state['right']['holding'] == 0 and not state['right']['dropped'], [state['right']['closed'], state['right']['holding'], state['right']['dropped']])

        at(HOLD_UNTIL_MS / 1000 + SETTLE_SECONDS)
        bench.place_hair_at_hand('left')
        moved = ingame.wait_until(lambda: (lambda s: s['left']['higgsDisabled'] and not s['right']['higgsDisabled'])(hands(bench)[0]), 4)
        state, grip = hands(bench)
        checks.check('HIGGS is handed back when the hair leaves, and taken from the hand it moved to', bool(moved), [state['left']['higgsDisabled'], state['right']['higgsDisabled']])

        ingame.stop_tracked_sequence(bench, token)
        token = None
        unheld = unheld_strand_move(bench, actor, held_bones)
        unheld_move = statistics.median(unheld) if unheld else 1e9
        checks.check('the same strands stay put when the hand moves with no grip', bool(unheld) and unheld_move <= MAXIMUM_UNHELD_MOVE, [round(m, 2) for m in unheld])
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
