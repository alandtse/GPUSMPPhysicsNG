"""Measures what the weather wind does to real claimed hair: the same frozen character with the wind strength at 0 and then raised.

    python tools/hair-tune/wind_probe.py [--port 8921] [--strengths 0,2,60] [--frames 150]

Needs a running game with the plugin, an exterior save, devbench answering and the lock held. Bone positions are read
in the head's frame (recordBones), so the character's own motion does not move them. For each strength the script
prints the claim's wind exposure and how far the mean hair pose sits from the pose with no wind.
"""

import argparse
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
            claim = next((c for c in bench.status()['claims'] if c['actorFormID'] == actor), {})
            poses.append((strength, mean_pose(bench, actor, args.frames)))
            print(f"strength {strength}: wind vector {[round(v, 1) for v in claim.get('windVector', [])]} exposure {claim.get('windFactor', 0):.2f}")
    finally:
        bench.set_settings(WindStrength=2.0)
        bench.command('clearTestCrowd')
    base_strength, base = poses[0]
    for strength, pose in poses[1:]:
        moved = [math.dist(a, b) for a, b in zip(base, pose)]
        print(f'strength {strength} vs the first run at {base_strength}: mean bone shift {sum(moved) / len(moved):.3f}, max {max(moved):.3f} units over {len(moved)} bones')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
