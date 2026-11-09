"""Measures what the weather wind does to real claimed hair: the same frozen character with the wind strength at 0 and then raised.

    python tools/hair-tune/wind_probe.py [--port 8921] [--strengths 0,2,60] [--frames 150]

Needs a running game with the plugin, an exterior save, devbench answering and the lock held. Bone positions are read
in the head's frame (recordBones), so the character's own motion does not move them. For each strength the script
prints the claim's wind exposure and how far the mean hair pose sits from the pose with no wind.
"""

import argparse
import json
import math
import time

import ingame

PREFIX = 'hdt'


def mean_pose(bench, actor, frames):
    bench.command('recordBones', actorFormID=actor, prefix=PREFIX, frames=frames)
    time.sleep(1.0)
    deadline = time.time() + 90
    while time.time() < deadline and bench.query('recordedBones').get('active'):
        time.sleep(1.0)
    recording = bench.query('recordedBones')
    rows = [frame['bones'] for frame in recording['frames']]
    count = len(rows[0])
    return [[sum(row[i][axis] for row in rows) / len(rows) for axis in range(3)] for i in range(count)]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=8921)
    parser.add_argument('--strengths', default='0,2,60')
    parser.add_argument('--frames', type=int, default=150)
    parser.add_argument('--hdt', action='store_true', help='also record the hair under hdtSMP64 (the plugin off)')
    parser.add_argument('--out', help='write the mean poses to this JSON file')
    parser.add_argument('--against', help='a JSON file from another launch: report each arm\'s shift against the same arm there')
    args = parser.parse_args()
    strengths = [float(value) for value in args.strengths.split(',')]

    bench = ingame.Bench(args.port)
    bench.set_settings(Enabled=True, CollisionEnabled=True, SolverBackend=1)
    bench.command('spawnTestCrowd', count=1)
    if not ingame.wait_until(lambda: bench.status().get('claimCount', 0) >= 1, 60):
        print('no claims after the crowd spawned')
        return 1
    time.sleep(3)
    bench.command('reevaluate')
    time.sleep(8)
    actor = bench.query('testCrowdOutfit')[0]['actorFormID']
    status = bench.status()
    print('wind', status.get('wind'))
    poses = []
    try:
        for strength in strengths:
            bench.set_settings(WindStrength=strength)
            time.sleep(8)
            fields = []

            def sample_field():
                claim = next((c for c in bench.status()['claims'] if c['actorFormID'] == actor), {})
                if strength > 0:
                    # The sampled field is normalised 0 to 1 (gusts to 2); the plugin scales it by 300 per unit of strength.
                    fields.append(claim.get('windMagnitude', 0.0) / (300.0 * strength))
                return claim

            sample_field()
            poses.append((strength, mean_pose(bench, actor, args.frames)))
            claim = sample_field()
            level = f'field {min(fields):.2f} to {max(fields):.2f}' if fields else 'no wind'
            print(f"strength {strength}: wind vector {[round(v, 1) for v in claim.get('windVector', [])]} {level} exposure {claim.get('windFactor', 0):.2f}")
        if args.hdt:
            # hdtSMP64 simulates the hair while the plugin is off; its own wind comes from its configs.json.
            bench.set_settings(Enabled=False)
            time.sleep(8)
            poses.append(('hdtSMP64', mean_pose(bench, actor, args.frames)))
            print('hdtSMP64 recorded')
    finally:
        bench.set_settings(Enabled=True, WindStrength=2.0)
        bench.command('clearTestCrowd')
    base_strength, base = poses[0]
    for strength, pose in poses[1:]:
        if strength == 'hdtSMP64':
            continue
        moved = [math.dist(a, b) for a, b in zip(base, pose)]
        print(f'strength {strength} vs the first run at {base_strength}: mean bone shift {sum(moved) / len(moved):.3f}, max {max(moved):.3f} units over {len(moved)} bones')
    if args.out:
        with open(args.out, 'w') as handle:
            json.dump({str(label): pose for label, pose in poses}, handle)
    if args.against:
        with open(args.against) as handle:
            other = json.load(handle)
        for label, pose in poses:
            if str(label) in other and len(other[str(label)]) == len(pose):
                moved = [math.dist(a, b) for a, b in zip(other[str(label)], pose)]
                print(f'{label} against the same arm in {args.against}: mean bone shift {sum(moved) / len(moved):.3f}, max {max(moved):.3f} units over {len(moved)} bones')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
