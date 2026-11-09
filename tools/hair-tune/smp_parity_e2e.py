"""In-game parity gate: our hair solver against hdtSMP64 on the same wig, turned the same way.

    python tools/hair-tune/smp_parity_e2e.py --data-dir <game Data folder> [--port 8920] [--crowd 2] [--runs 2] [--update-baseline]

Needs a game that is already running with the devbench server answering, a save loaded, hdtSMP64 installed, and
you holding the game's lock. The script spawns frozen test characters with a wig, turns one of them while it
records the wig's and the cape's bones in the actor's frame, once per solver: hdtSMP64 (GPU SMP Physics off) and the rigid solver.
The rigid solver is scored by how far its mean bone positions sit from the centroid of hdtSMP64's
runs, and by the spread of its motion relative to hdtSMP64's; hdtSMP64's own run-to-run distance sets the noise floor.

The limits live in smp_parity_baseline.json, next to this script, one set per game. They are measured values with headroom, so a change
that moves a solver away from hdtSMP64 fails here. After a deliberate change, run with --update-baseline on a scene you
trust and commit the new file. Exit code 0 when every solver is within its limits.
"""

import argparse
import json
import pathlib
import re
import sys
import threading
import time

import ingame
import numpy as np

BASELINE = pathlib.Path(__file__).with_name('smp_parity_baseline.json')
FRAMES = 420
RECORD_SECONDS = 14
SETTLE_SECONDS = 12
DISTANCE_HEADROOM = 1.25   # a solver may sit this much farther from hdtSMP64 than the baseline run did
DISTANCE_SLACK = 0.15      # units, added to that so a near-exact baseline does not fail on rounding
SD_RATIO_BAND = 0.35       # how far past the baseline's observed range a solver's motion spread may fall, as a fraction of it
SD_RATIO_SLACK = 0.05      # added to both ends, so a solver that barely moves does not fail on rounding
ROOT_NODE = 'NPC Spine2 [Spn2]'
PREFIXES = ('HG_ColdHeart', 'hdtSSEPhysics_AutoRename', 'hdtSSE', 'HDT', 'CME')
SOLVERS = ('rigid',)
NOT_A_WIG = ('cape', 'cloak', 'cloth', 'skirt', 'tail')
CLOTH = ('cape', 'cloak')
MOVING_SPREAD = 0.01       # units; a bone that moves less than this under hdtSMP64 is kinematic and says nothing about a solver
MINIMUM_BONES = 3


def load_positions(recording):
    """Each recorded bone's positions in the root node's frame, as an array of frames by three, keyed by bone name."""
    names = recording['names']
    world = np.array([frame['world'] for frame in recording['frames']])
    root = world[:, 0]
    rotation = root[:, 3:12].reshape(-1, 3, 3)
    origin = root[:, :3]
    return {names[i]: np.einsum('fji,fj->fi', rotation, world[:, i, :3] - origin) for i in range(1, len(names))}


def bone_names(data_dir, physics_file):
    """The bone names an hdtSMP64 physics file declares, which are the names of the nodes it moves."""
    text = (pathlib.Path(data_dir) / physics_file.replace('\\', '/')).read_text(errors='replace')
    return set(re.findall(r'<bone name="([^"]+)"', text))


def only(runs, names, moving):
    """The runs cut down to the bones a physics file declares that also move under hdtSMP64.

    A recorded node is named "<prefix>_<slot> <bone name>", so the bone name is what follows the first space.
    """
    return [{bone: path for bone, path in run.items() if bone in moving and bone.split(' ', 1)[-1] in names} for run in runs]


def moving_bones(smp):
    """The bones whose position varies by more than MOVING_SPREAD in any hdtSMP64 run."""
    return {bone for bone in smp[0] if any(run[bone].std(0).max() > MOVING_SPREAD for run in smp)}


def summarise(runs):
    """Per run, the mean and the spread of every bone's position, as arrays of bones by three."""
    bones = sorted(runs[0])
    mean = [np.array([run[bone].mean(0) for bone in bones]) for run in runs]
    spread = [np.array([run[bone].std(0) for bone in bones]) for run in runs]
    return mean, spread


def score(smp, solver_runs):
    """The solver's mean distance from the hdtSMP64 centroid, the noise floor, and the motion spread ratio."""
    smp_mean, smp_spread = summarise(smp)
    mean, spread = summarise(solver_runs)
    centroid = np.mean(smp_mean, axis=0)
    floor = float(np.mean([np.linalg.norm(smp_mean[i] - np.mean([m for j, m in enumerate(smp_mean) if j != i], axis=0), axis=1).mean()
                           for i in range(len(smp_mean))]))
    distance = float(np.mean([np.linalg.norm(m - centroid, axis=1).mean() for m in mean]))
    ratio = float((np.mean(spread, axis=(0, 1)) / np.mean(smp_spread, axis=(0, 1))).mean())
    return {'distance': distance, 'smpNoise': floor, 'sdRatio': ratio}


def check_limits(checks, name, measured, limit):
    ceiling = limit['distance'] * DISTANCE_HEADROOM + DISTANCE_SLACK
    checks.check(f'{name} stays near hdtSMP64', measured['distance'] <= ceiling,
                 f"distance {measured['distance']:.2f} (limit {ceiling:.2f}, hdtSMP64 against itself {measured['smpNoise']:.2f})")
    low, high = limit['sdLow'] * (1.0 - SD_RATIO_BAND) - SD_RATIO_SLACK, limit['sdHigh'] * (1.0 + SD_RATIO_BAND) + SD_RATIO_SLACK
    checks.check(f'{name} moves as much as hdtSMP64', low <= measured['sdRatio'] <= high,
                 f"spread ratio {measured['sdRatio']:.2f} (allowed {low:.2f} to {high:.2f})")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=8920, help='8920 for SE, 8921 for VR')
    parser.add_argument('--crowd', type=int, default=2)
    parser.add_argument('--data-dir', required=True, help='the game\'s Data folder, to read the physics files from')
    parser.add_argument('--runs', type=int, default=2, help='recordings per solver; hdtSMP64 always gets at least two for its noise floor')
    parser.add_argument('--update-baseline', action='store_true', help='widen the baseline to take in this run instead of checking it; run it on several launches')
    parser.add_argument('--reset-baseline', action='store_true', help='with --update-baseline, start the baseline for this game over from this run')
    arguments = parser.parse_args()
    args_reset = arguments.reset_baseline

    bench = ingame.Bench(arguments.port)
    game = 'VR' if bench.post('inspect', {'kind': 'state'}).get('vr') else 'SE'
    print(f'game: {game}', flush=True)
    checks = ingame.Checks()
    bench.set_settings(Enabled=True, ClaimArmor=True, ClaimHeadParts=True, RetargetRest=False)
    bench.command('clearTestCrowd')
    bench.command('stripTestCrowd', enabled=True)
    bench.command('setTestCrowdCloak', index=3)
    time.sleep(2)
    bench.command('spawnTestCrowd', count=arguments.crowd)
    ingame.wait_until(lambda: bench.status().get('claimCount', 0) >= arguments.crowd, 150)
    time.sleep(10)

    claims = [c for c in bench.status().get('claims', []) if c.get('actorFormID') != 0x14]
    wigs = sorted((c for c in claims if not any(word in str(c.get('file')).lower() for word in NOT_A_WIG)), key=lambda c: -c.get('boneCount', 0))
    if not checks.check('a crowd actor has a wig claim', bool(wigs), [(hex(c.get('actorFormID', 0)), c.get('file'), c.get('boneCount')) for c in claims]):
        return checks.finish()
    actor, wig_file = wigs[0]['actorFormID'], wigs[0]['file']
    cloths = [c for c in claims if c.get('actorFormID') == actor and any(word in str(c.get('file')).lower() for word in CLOTH)]
    targets = {'wig': bone_names(arguments.data_dir, wig_file)}
    if cloths:
        targets['cape'] = bone_names(arguments.data_dir, cloths[0]['file'])
    print(f"wig: {wig_file} on {actor:#x}, cape: {cloths[0]['file'] if cloths else 'none'}", flush=True)
    counts = {p: len(bench.query('sampleBones', actorFormID=actor, prefix=p).get('bones', [])) for p in PREFIXES}
    prefix = max(counts, key=counts.get)
    if not checks.check('the wig bones can be sampled', counts[prefix] > 0, counts):
        return checks.finish()

    def record():
        bench.command('recordBones', actorFormID=actor, prefix=prefix, frames=FRAMES, world=True, extraNodes=[ROOT_NODE])
        stop = threading.Event()

        def turn():
            step = 0
            while not stop.is_set():
                try:
                    bench.command('turnTestCrowd', index=0, degrees=(step * 15) % 360)
                except Exception:
                    pass
                step += 1
                time.sleep(0.12)

        thread = threading.Thread(target=turn)
        thread.start()
        time.sleep(RECORD_SECONDS)
        stop.set()
        thread.join()
        recording = bench.query('recordedBones')
        recording = recording.get('result', recording)
        return load_positions(recording)

    def resettle():
        bench.set_settings(Enabled=False)
        time.sleep(3)
        bench.set_settings(Enabled=True)
        time.sleep(8 + SETTLE_SECONDS)

    results = {}
    for solver in SOLVERS:
        resettle()
        mine = [c for c in bench.status().get('claims', []) if c.get('actorFormID') == actor and c.get('file') == wig_file]
        checks.check(f'{solver} runs the wig on the GPU', bool(mine) and all(c.get('gpuRegistered') for c in mine), [c.get('gpuRegistered') for c in mine])
        resettle()
        results[solver] = [record() for _ in range(arguments.runs)]

    bench.set_settings(Enabled=False)
    time.sleep(4)
    smp = [record() for _ in range(max(arguments.runs, 2))]
    bench.set_settings(Enabled=True)

    moving = moving_bones(smp)
    for target, names in list(targets.items()):
        count = len(only(smp[:1], names, moving)[0])
        if not checks.check(f'the {target} has bones that move', count >= MINIMUM_BONES, f'{count} bones'):
            del targets[target]
    measured = {target: {solver: score(only(smp, names, moving), only(runs, names, moving)) for solver, runs in results.items()} for target, names in targets.items()}
    for target, scores in measured.items():
        for solver, value in scores.items():
            print(f"{target} {solver}: distance {value['distance']:.2f}, spread ratio {value['sdRatio']:.2f}, hdtSMP64 noise {value['smpNoise']:.2f}", flush=True)

    if arguments.update_baseline:
        baseline = json.loads(BASELINE.read_text()) if BASELINE.exists() else {}
        for target, scores in measured.items():
            for solver, value in scores.items():
                entry = baseline.setdefault(game, {}).setdefault(target, {}).get(solver)
                if args_reset or entry is None:
                    entry = {'distance': value['distance'], 'sdLow': value['sdRatio'], 'sdHigh': value['sdRatio']}
                else:
                    entry = {'distance': max(entry['distance'], value['distance']), 'sdLow': min(entry['sdLow'], value['sdRatio']), 'sdHigh': max(entry['sdHigh'], value['sdRatio'])}
                baseline[game][target][solver] = {k: round(v, 3) for k, v in entry.items()}
        BASELINE.write_text(json.dumps(baseline, indent=2) + '\n')
        print(f'wrote {BASELINE.name}', flush=True)
        return checks.finish()

    limits = json.loads(BASELINE.read_text())[game]
    for target, scores in measured.items():
        for solver in SOLVERS:
            check_limits(checks, f'{target} {solver}', scores[solver], limits[target][solver])
    return checks.finish()


if __name__ == '__main__':
    sys.exit(main())
