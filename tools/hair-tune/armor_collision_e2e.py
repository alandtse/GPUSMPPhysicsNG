"""Armor collision test on a game that is already running with the plugin: does a claimed armor's physics collide?

    GPUSMP_STANDALONE=1 python tools/hair-tune/armor_collision_e2e.py [--port 8920] [--armor 0x000D8D50] [--file forsworn.xml]

Dresses a male and a female test character in the same body armor, then on the male one's claim of the armor's
physics file it measures the rig's bone positions (solverBones, in the kinematic root's frame). A skirt keeps
swinging by itself, so every push is compared with the swing measured just before it in the same state:
  far probe (negative control)        a capsule out of reach moves the bones no more than the swing does
  collision off                       the plugin declines the file, since its collision shapes cannot be honoured
  near probe on each side             bones move well beyond the swing, and the two sides move mostly different bones
  smaller probe radius                moves them less than the larger one
  release                             bones return toward where they were within six seconds
  made by equipping                   a claim created by equipping the armor is as steady at rest as a re-offered one
The female character is reported, not asserted: which claims a body armor makes depends on the mesh the plugin loads.
Thresholds are game units: the swing is never taken below SWING_FLOOR, a push must beat PUSH_FACTOR times the swing plus
PUSH_MARGIN, a negative control may reach NEGATIVE_FACTOR times it, a steady rig swings under STEADY_LIMIT, and at most
SHARED_MAX of each side's five most-moved bones may be shared.
Takes about three minutes. Needs the game lock, a loaded save and the player in an open area; always run it to the
end, since collision is switched off during one step (Ctrl-C restores it). Exit code 0 when every check passes.
"""

import argparse
import math
import sys
import time

import ingame

SWING_FLOOR = 0.5
PUSH_FACTOR = 3.0
PUSH_MARGIN = 1.0
NEGATIVE_FACTOR = 2.0
STEADY_LIMIT = 2.5
SETTLE_LIMIT = 2.5
SETTLE_SECONDS = 40
PROBE_RADIUS = 35
SMALL_RADIUS = 8
PROBE_SIDE = 35
SHARED_MAX = 2


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=8920)
    parser.add_argument('--armor', default='0x000D8D50', help='body armor form ID to wear')
    parser.add_argument('--file', default='forsworn.xml', help='physics file, as the end of its path, the armor is expected to claim')
    parser.add_argument('--male', default='Hadvar')
    parser.add_argument('--female', default='Lydia')
    parser.add_argument('--setup-only', action='store_true', help='stop once the claim is up and leave the characters standing')
    parser.add_argument('--keep', action='store_true', help='leave the characters standing afterwards, for looking at them')
    args = parser.parse_args()

    bench = ingame.Bench(args.port, standalone=True)
    checks = ingame.Checks()
    wanted = args.file.lower()

    def clear_menus():
        for _ in range(6):
            blocked = bench.post('wait', {'ms': 200}).get('blockedBy', [])
            if not blocked:
                return
            for menu in blocked:
                if menu['name'] == 'MessageBoxMenu':
                    bench.post('menu', {'action': 'accept', 'index': 1})
                else:
                    bench.post('menu', {'action': 'close', 'name': menu['name']})
            time.sleep(0.5)

    def console(command):
        clear_menus()
        bench.post('console', {'command': command})

    def base_of(name):
        forms = bench.post('forms', {'action': 'find', 'formType': 'NPC', 'name': name, 'limit': 5}).get('forms', [])
        return next((form['formId'] for form in forms if form.get('plugin') == 'Skyrim.esm'), None)

    def is_wanted(path):
        return path.lower().replace(chr(92), '/').endswith(wanted)

    def claim_of(actor):
        return next((c for c in bench.status().get('claims', []) if c['actorFormID'] == actor and is_wanted(c['file'])), None)

    def files_of(actor):
        return sorted({c['file'] for c in bench.status().get('claims', []) if c['actorFormID'] == actor})

    def bones(actor):
        for claim in bench.query('solverBones', actorFormID=actor).get('claims', []):
            if claim.get('actorFormID') == actor and is_wanted(claim.get('file', '')):
                return {bone['name']: bone['local'] for bone in claim['bones'] if bone.get('finite') and 'local' in bone}
        return {}

    def shift(before, after):
        distances = [math.dist(before[name], after[name]) for name in before if name in after]
        return sum(distances) / len(distances) if distances else float('nan')

    def most_moved(before, after, count=5):
        """The names of the bones that moved furthest."""
        moved = sorted(((math.dist(before[n], after[n]), n) for n in before if n in after), reverse=True)
        return {name for _, name in moved[:count]}

    def window(actor, count=4, gap=0.4):
        samples = []
        for _ in range(count):
            samples.append(bones(actor))
            time.sleep(gap)
        return samples

    def swing_of(samples):
        return max(shift(samples[0], later) for later in samples[1:])

    def probe(actor, offset, radius, enabled=True):
        bench.command('probeBodyContact', actorFormID=actor, enabled=enabled, offset=offset, length=60, radius=radius)

    def push(actor, offset, radius):
        """The bones' swing without the probe, then their shift from where they were with it on, in the same state."""
        probe(actor, offset, radius, enabled=False)
        time.sleep(1.0)
        quiet = window(actor)
        swing = swing_of(quiet)
        before = quiet[-1]
        probe(actor, offset, radius)
        time.sleep(1.2)
        pushed = window(actor, 3)
        peak = max(pushed, key=lambda sample: shift(before, sample))
        moved = shift(before, peak)
        top = most_moved(before, peak)
        probe(actor, offset, radius, enabled=False)
        time.sleep(6.0)
        back = shift(before, bones(actor))
        return swing, moved, top, back

    try:
        bench.set_settings(Enabled=True, CollisionEnabled=True, ClaimArmor=True)
        bench.command('clearTestCrowd')
        clear_menus()
        male_base, female_base = base_of(args.male), base_of(args.female)
        if not checks.check('both characters exist in the game data', bool(male_base and female_base), (male_base, female_base)):
            return checks.finish()
        bench.command('spawnTestCrowd', count=2, bases=[male_base, female_base])
        bench.command('stripTestCrowd', enabled=True)
        time.sleep(4)
        ids = [row['actorFormID'] for row in bench.query('testCrowdOutfit')]
        if not checks.check('two test characters stand', len(ids) == 2, ids):
            return checks.finish()
        male, female = ids

        for actor in (male, female):
            console(f'{actor:08X}.additem {int(args.armor, 16):08X} 1')
            console(f'{actor:08X}.equipitem {int(args.armor, 16):08X} 0')
        time.sleep(4)
        worn_now = {row['actorFormID']: row.get('worn', []) for row in bench.query('testCrowdOutfit')}
        for label, actor in (('male', male), ('female', female)):
            checks.check(f'the {label} character wears the armor', any('orsworn' in str(item) for item in worn_now.get(actor, [])),
                         worn_now.get(actor))

        if not checks.check(f'the male character claims {args.file}', ingame.wait_until(lambda: claim_of(male), 30) is not None,
                            files_of(male)):
            return checks.finish()
        print(f'male claims:   {files_of(male)}')
        print(f'female claims: {files_of(female)}  (reported, not asserted)', flush=True)
        row = claim_of(male)
        checks.check('the claim is on the GPU and not culled',
                     not row.get('culledByBudget') and not row.get('culledOffscreen') and not row.get('culledSmall') and not row.get('inactive'),
                     {k: row.get(k) for k in ('cameraDistance', 'inactive', 'culledByBudget', 'culledOffscreen', 'culledSmall')})
        first = row.get('steppedFrames', 0)
        time.sleep(2)
        checks.check('the claim is stepping', claim_of(male).get('steppedFrames', 0) > first, (first, claim_of(male).get('steppedFrames')))
        checks.check('the rig has dynamic bones and collision spheres', len(bones(male)) > 0 and row.get('sphereCount', 0) > 0,
                     (len(bones(male)), row.get('sphereCount')))

        if args.setup_only:
            args.keep = True
            return checks.finish()
        probe(male, [0, 0, 0], PROBE_RADIUS, enabled=False)
        time.sleep(8)
        made_by_equip = swing_of(window(male))
        checks.check('a claim made by equipping is steady at rest', made_by_equip <= STEADY_LIMIT, f'swing {made_by_equip:.2f}')
        bench.set_settings(Enabled=False)
        ingame.wait_until(lambda: claim_of(male) is None, 20)
        time.sleep(2)
        bench.set_settings(Enabled=True)
        ingame.wait_until(lambda: claim_of(male), 30)
        time.sleep(8)
        started = time.time()
        swing = float('inf')
        curve = []
        while swing > SETTLE_LIMIT and time.time() - started < SETTLE_SECONDS:
            swing = swing_of(window(male))
            curve.append(f'{time.time() - started:.0f}s:{swing:.1f}')
        print('  swing over time ' + ' '.join(curve), flush=True)
        print(f'  swing {swing:.2f} after {time.time() - started:.0f} s ({"settled" if swing <= SETTLE_LIMIT else "still swinging"})', flush=True)

        swing, far, _, _ = push(male, [400, 0, 0], PROBE_RADIUS)
        checks.check('a far probe moves the bones no more than the swing does', far <= NEGATIVE_FACTOR * max(swing, SWING_FLOOR) + PUSH_MARGIN,
                     f'{far:.2f} against swing {swing:.2f}')

        rounds = []
        for round_number in range(3):
            plus = push(male, [PROBE_SIDE, 0, 0], PROBE_RADIUS)
            minus = push(male, [-PROBE_SIDE, 0, 0], PROBE_RADIUS)
            shared = len(plus[2] & minus[2])
            rounds.append((plus, minus, shared))
            print(f'round {round_number + 1}: +x moved {plus[1]:.2f} (swing {plus[0]:.2f}, back {plus[3]:.2f})  '
                  f'-x moved {minus[1]:.2f} (swing {minus[0]:.2f}, back {minus[3]:.2f})  bones moved most in common {shared} of 5', flush=True)

        def beats_swing(result):
            return result[1] > PUSH_FACTOR * max(result[0], SWING_FLOOR) + PUSH_MARGIN

        checks.check('a near probe moves the bones well beyond the swing, on both sides, every round',
                     all(beats_swing(r[0]) and beats_swing(r[1]) for r in rounds),
                     [(round(r[0][1], 1), round(r[1][1], 1)) for r in rounds])
        checks.check('the two sides move mostly different bones, every round', all(r[2] <= SHARED_MAX for r in rounds), [r[2] for r in rounds])
        checks.check('the bones return after the probe leaves', all(r[0][3] < 0.5 * r[0][1] and r[1][3] < 0.5 * r[1][1] for r in rounds),
                     [(round(r[0][3], 1), round(r[1][3], 1)) for r in rounds])

        small = push(male, [PROBE_SIDE, 0, 0], SMALL_RADIUS)
        checks.check('a smaller probe moves the bones less than the larger one', small[1] < rounds[0][0][1],
                     f'{small[1]:.2f} against {rounds[0][0][1]:.2f}')

        bench.set_settings(CollisionEnabled=False)
        gone = ingame.wait_until(lambda: claim_of(male) is None, 30)
        checks.check('with collision off the plugin leaves the armor to hdtSMP64', gone is not None, files_of(male))
        bench.set_settings(CollisionEnabled=True)
    finally:
        try:
            bench.set_settings(CollisionEnabled=True, Enabled=True)
            if not args.keep:
                bench.command('stripTestCrowd', enabled=False)
                bench.command('clearTestCrowd')
        except Exception as error:
            print('cleanup failed:', error)
    return checks.finish()


if __name__ == '__main__':
    sys.exit(main())
