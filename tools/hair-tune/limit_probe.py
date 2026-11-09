"""Holds real hair with the headset-free pin, lets go, and measures the joints against their authored limits.

    python tools/hair-tune/limit_probe.py [--port 8921] [--hold 4.0] [--count 3] [--reach 30]

Needs a running game with the plugin, devbench answering, a save loaded and the lock held. It spawns a frozen test
character, records every simulated joint's angles with recordBones, holds `count` strands `reach` units off with
holdStrand, releases them, and reports from the recording:

  during the hold -> how far past its limits the worst joint went (a hold frees the joints above it)
  after the release -> the fastest a joint moved back, in rad/s, and how long it took to get inside its limits

The release should not move a joint faster than about kLimitReturnRate (8 rad/s) while it is outside its limits.
"""

import argparse
import math
import time

import ingame

PREFIX = 'hdt'


def excess(angle, lower, upper):
    if lower > upper:
        return 0.0
    return max(lower - angle, angle - upper, 0.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=8921)
    parser.add_argument('--hold', type=float, default=4.0)
    parser.add_argument('--count', type=int, default=3)
    parser.add_argument('--reach', type=float, default=30.0)
    parser.add_argument('--frames', type=int, default=900)
    args = parser.parse_args()

    bench = ingame.Bench(args.port)
    bench.set_settings(Enabled=True, CollisionEnabled=True, SolverBackend=1)
    bench.command('spawnTestCrowd', count=1)
    if not ingame.wait_until(lambda: bench.status().get('claimCount', 0) >= 1, 60):
        print('no claims after the crowd spawned')
        return 1
    time.sleep(3)
    bench.command('reevaluate')
    time.sleep(8)
    outfit = bench.query('testCrowdOutfit')
    actor = outfit[0]['actorFormID']
    print('actor', hex(actor))

    bench.command('recordBones', actorFormID=actor, prefix=PREFIX, frames=args.frames, joints=True)
    started = time.time()
    time.sleep(1.5)
    bench.command('holdStrand', actorFormID=actor, count=args.count, offset=[0.0, args.reach, 0.0], enabled=True)
    held_at = time.time() - started
    time.sleep(args.hold)
    bench.command('holdStrand', actorFormID=actor, enabled=False)
    released_at = time.time() - started
    deadline = started + 60
    while time.time() < deadline and bench.query('recordedBones').get('active'):
        time.sleep(1.0)
    recording = bench.query('recordedBones')
    bench.command('clearTestCrowd')

    info = recording.get('jointInfo') or {}
    frames = recording.get('frames') or []
    joints = [{'lower': lower, 'upper': upper} for lower, upper in zip(info.get('lower', []), info.get('upper', []))]
    if not frames or not joints:
        print('keys', list(recording.keys()), 'jointInfo keys', list(info.keys()), 'frame keys', list(frames[0].keys()) if frames else None)
        return 1
    keys = list(frames[0].keys())
    clock = next((key for key in ('time', 't', 'timestamp', 'seconds') if key in keys), None)
    angle_key = next((key for key in ('joints', 'jointAngles', 'angles') if key in keys), None)
    if not clock or not angle_key:
        print('frame keys', keys)
        return 1
    first = frames[0][clock]
    print(f"recording clock '{clock}' spans {frames[-1][clock] - first:.2f}, frameCount {recording.get('frameCount')}, active {recording.get('active')}")
    print(f'{len(frames)} frames, {len(joints)} joints, held at {held_at:.1f}s released at {released_at:.1f}s')

    def limits(joint):
        lower = joint.get('lower')
        upper = joint.get('upper')
        return lower, upper

    worst_hold = 0.0
    worst_rate = 0.0
    recovered = []
    previous = None
    for frame in frames:
        t = frame[clock] - first
        angles = frame.get(angle_key) or []
        if len(angles) != len(joints):
            continue
        over = []
        for joint, angle in zip(joints, angles):
            lower, upper = limits(joint)
            over.append(max(excess(angle[axis], lower[axis], upper[axis]) for axis in range(3)))
        if held_at <= t < released_at:
            worst_hold = max(worst_hold, max(over))
        if t >= released_at and previous is not None:
            dt = t - previous[0]
            if dt > 0:
                for index, (joint, angle) in enumerate(zip(joints, angles)):
                    if previous[2][index] > 1e-3:
                        step = max(abs(angle[axis] - previous[1][index][axis]) for axis in range(3))
                        worst_rate = max(worst_rate, step / dt)
        if t >= released_at and max(over) <= 1e-3 and not recovered:
            recovered.append(t - released_at)
        if t >= released_at and int((t - released_at) * 2) != int(((previous[0] if previous else t) - released_at) * 2):
            outside = sum(1 for value in over if value > 1e-3)
            print(f'  +{t - released_at:.1f}s after release: worst excess {max(over):.3f} rad, {outside} of {len(over)} joints outside')
        previous = (t, angles, over)
    print(f'worst joint excess past its limits while held: {worst_hold:.2f} rad')
    print(f'fastest a joint outside its limits moved after the release: {worst_rate:.1f} rad/s (return rate {8.0})')
    print('all joints inside their limits after the release in', f'{recovered[0]:.2f} s' if recovered else 'not within the recording')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
