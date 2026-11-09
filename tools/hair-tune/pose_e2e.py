"""Test crowd and debug overlay checks that need no controllers: works in SE and in VR.

    python tools/hair-tune/pose_e2e.py [--port 8920] [--crowd 3]

Needs a game that is already running with the devbench server answering, a save loaded, and you holding its lock.
The script turns GPU SMP Physics on, spawns frozen test characters and checks:

  outfits    -> strip leaves only wigs, armor cycling dresses everyone, restore puts the originals back
  animation  -> the default Cicero idle plays on every character, and ResetRoot stops it
  overlays   -> the debug line count follows the actor limit and stays inside the line budget
  pose       -> pulling a strand moves the solver's pose away from the animation pose, which the overlays and the grab read

Exit code 0 when every check passes.
"""

import argparse
import sys
import time

import ingame

SETTLE_SECONDS = 4.0
LINE_BUDGET = 65536       # the debug line renderer's line limit
PULLED_DISPLACEMENT = 3.0  # game units a pulled hair bone must sit from its animation pose
PULL_OFFSET = [20.0, 0.0, 10.0]


def worn_rows(bench):
    return bench.query('testCrowdOutfit')


def debug_lines(bench):
    lines = bench.status().get('debugLines', {})
    return lines.get('requested', 0), lines.get('drawn', 0), lines.get('stateFailures', 0)


def max_displacement(bench, actor):
    rows = [row for row in bench.query('poseCheck', actorFormID=actor) if row.get('dynamicBones', 0) > 0]
    return max((row['maxSolvedFromRest'] for row in rows), default=0.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=8920)
    parser.add_argument('--crowd', type=int, default=3)
    args = parser.parse_args()

    bench = ingame.Bench(args.port)
    checks = ingame.Checks()
    try:
        bench.set_settings(Enabled=True, CollisionEnabled=True, SolverBackend=1, BodyContacts=True,
                           ShowColliderDebug=False, ShowJointDebug=False, ShowSkeletonDebug=False, ShowGrabDebug=False)
        bench.command('spawnTestCrowd', count=args.crowd)
        if not ingame.wait_until(lambda: len(worn_rows(bench)) == args.crowd, 60):
            checks.check('test characters stand', False, len(worn_rows(bench)))
            return checks.finish()
        time.sleep(SETTLE_SECONDS * 2)
        bench.command('reevaluate')
        time.sleep(SETTLE_SECONDS)

        rows = worn_rows(bench)
        checks.check('the crowd starts dressed and wigged', all(row['worn'] and row['kept'] for row in rows), [(len(r['worn']), len(r['kept'])) for r in rows])

        bench.command('stripTestCrowd', enabled=True)
        stripped = ingame.wait_until(lambda: all(not row['worn'] for row in worn_rows(bench)), 15)
        rows = worn_rows(bench)
        checks.check('strip leaves only wigs', bool(stripped) and all(row['kept'] for row in rows), [(len(r['worn']), len(r['kept'])) for r in rows])

        bench.command('cycleTestCrowdArmor', step=1)
        dressed = ingame.wait_until(lambda: all(row['worn'] for row in worn_rows(bench)), 15)
        report = str(bench.query('testCrowdReport').get('report', ''))
        checks.check('armor cycling dresses every character and keeps the wigs', bool(dressed) and all(row['kept'] for row in worn_rows(bench)), report)

        bench.command('stripTestCrowd', enabled=False)
        restored = ingame.wait_until(lambda: all(row['worn'] for row in worn_rows(bench)), 15)
        checks.check('restore puts the outfits back', bool(restored), [len(r['worn']) for r in worn_rows(bench)])

        idles = bench.query('testCrowdAnimations', filter='cicero', limit=50)
        checks.check('the idle list includes Cicero\'s dance', any(entry.get('editorId') == 'IdleCiceroDance1' for entry in idles), len(idles))
        bench.command('playTestCrowdAnimation', idle='')
        time.sleep(2)
        report = str(bench.query('testCrowdReport').get('report', ''))
        checks.check('the default idle plays on every character', report.startswith('Played ') and f'on {args.crowd} of {args.crowd}' in report, report)
        bench.command('playTestCrowdAnimation', idle='ResetRoot')
        time.sleep(2)
        report = str(bench.query('testCrowdReport').get('report', ''))
        checks.check('ResetRoot stops the animation', report.startswith('Played ResetRoot'), report)

        bench.set_settings(**({'developerMode': True} if bench.standalone else {}), ShowColliderDebug=True, ShowJointDebug=True, ShowSkeletonDebug=True, DebugMaxActors=1)
        time.sleep(SETTLE_SECONDS)
        one, drawn_one, failures = debug_lines(bench)
        bench.set_settings(DebugMaxActors=args.crowd)
        time.sleep(SETTLE_SECONDS)
        many, drawn_many, failures_after = debug_lines(bench)
        checks.check('the overlays draw lines', one > 0, one)
        checks.check('more actors draw more lines', many > one, [one, many])
        checks.check('the drawn lines stay inside the budget', drawn_one == min(one, LINE_BUDGET) and drawn_many == min(many, LINE_BUDGET), [drawn_one, drawn_many])
        checks.check('the overlays always find the game render targets', failures_after == failures, [failures, failures_after])
        bench.set_settings(ShowColliderDebug=False, ShowJointDebug=False, ShowSkeletonDebug=False)

        claims = [claim for claim in bench.status().get('claims', []) if claim.get('actorFormID') in {row['actorFormID'] for row in worn_rows(bench)}]
        actor = claims[0]['actorFormID'] if claims else 0
        checks.check('a test character has claimed hair', actor != 0, len(claims))
        if actor:
            time.sleep(SETTLE_SECONDS)
            resting = max_displacement(bench, actor)
            bench.command('holdStrand', actorFormID=actor, count=3, offset=PULL_OFFSET)
            time.sleep(SETTLE_SECONDS)
            pulled = max_displacement(bench, actor)
            bench.command('holdStrand', actorFormID=actor, enabled=False)
            checks.check('pulling a strand moves the solver pose from the animation pose', pulled >= PULLED_DISPLACEMENT and pulled > resting, [round(resting, 2), round(pulled, 2)])
    finally:
        bench.command('clearTestCrowd')
    return checks.finish()


if __name__ == '__main__':
    sys.exit(main())
