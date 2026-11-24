"""Measure, per rendered frame, how far an actor's body skin shows in front of its worn cloth while it sprints.

Drives the plugin's patchProbe command over devbench on a running game (GSP_PORT, default 8920) and compares arms
in one session, so no screenshots are read. Each arm is a set of plugin settings; "hdt" switches the plugin off so
hdtSMP64 simulates the cloth and gives the reference. A frame counts as a pop when the skin is further in front of
the cloth than the reference arm ever gets (or --threshold when there is no reference arm).

    python tools/game/patch_probe.py --start F:/Temp/stairs_top.json --arms gpu,hdt --runs 6 --stride 8

--start is a JSON file {"pos": [x, y, z]} to teleport the player to before each sprint. --stride measures every Nth
skin vertex: the full probe costs tens of milliseconds a frame, which changes the frame rate under test.
"""

import argparse
import json
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'hair-tune'))
import ingame  # noqa: E402

PLAYER = 0x14
ARMS = {
    'gpu': {'Enabled': True, 'SameFrameResults': True},
    'gpu-late': {'Enabled': True, 'SameFrameResults': False},
    'gpu-nodeskin': {'GpuSkinning': False},
    'hdt': {'Enabled': False},
}
DEFAULTS = {'Enabled': True, 'GpuSkinning': True, 'SameFrameResults': False}
ROW = {'frame': 0, 'measured': 1, 'outside': 2, 'max': 3, 'lag': 5, 'bone': 6, 'drawnMax': 14, 'skinCaptured': 16, 'clothCaptured': 17, 'wallMs': 22, 'probeMs': 24}


def key(bench, action, name):
    body = {'action': action, 'device': 'keyboard', 'key': name}
    if action == 'down':
        body['maxHoldMs'] = 60000
    bench.post('input', body, 15)


def sprint(bench, position, seconds, stride, drawn, bones):
    key(bench, 'up', 'LeftAlt')
    key(bench, 'up', 'W')
    bench.post('papyrus', {'action': 'call', 'script': 'ObjectReference', 'function': 'SetPosition', 'self': {'form': hex(PLAYER)},
                           'args': [position[0], position[1], position[2] + 5]}, 15)
    time.sleep(4)
    bench.command('patchProbe', actorFormID=PLAYER, stop=True)
    time.sleep(0.2)
    started = bench.command('patchProbe', actorFormID=PLAYER, stride=stride, drawn=drawn, bones=bones)
    if isinstance(started, dict) and started.get('error'):
        raise SystemExit(f"patchProbe did not start: {started['error']}")
    time.sleep(0.5)
    try:
        key(bench, 'down', 'W')
        key(bench, 'down', 'LeftAlt')
        time.sleep(seconds)
    finally:
        key(bench, 'up', 'LeftAlt')
        key(bench, 'up', 'W')
    time.sleep(0.5)
    return bench.query('patchProbeRows').get('rows', [])


def number(value):
    return float(value) if isinstance(value, (int, float)) else 0.0


def depth(row):
    """How far the skin is in front of the cloth as drawn, when the frame's skin matrices were captured, else by the nodes."""
    captured = row[ROW['skinCaptured']] or row[ROW['clothCaptured']]
    return number(row[ROW['drawnMax'] if captured else ROW['max']])


def longest_streak(values, threshold):
    best = current = 0
    for value in values:
        current = current + 1 if value > threshold else 0
        best = max(best, current)
    return best


def summarize(name, runs, threshold):
    rows = [row for run in runs for row in run]
    depths = [depth(row) for row in rows]
    if not rows:
        print(f'{name:10s} no frames measured')
        return
    over = sum(depth > threshold for depth in depths)
    print(f"{name:10s} frames {len(rows):4d} | frame {statistics.median(number(r[ROW['wallMs']]) for r in rows):5.1f} ms"
          f" (probe {statistics.median(number(r[ROW['probeMs']]) for r in rows):4.1f}) | result age {statistics.median(number(r[ROW['lag']]) for r in rows):.0f}"
          f" | above {threshold:.2f}: {over:3d} ({100.0 * over / len(rows):4.1f}%) | max {max(depths):.2f}"
          f" | longest streak {max(longest_streak([depth(r) for r in run], threshold) for run in runs)} frames"
          f" | as drawn {100.0 * sum(bool(r[ROW['skinCaptured']] or r[ROW['clothCaptured']]) for r in rows) / len(rows):3.0f}%"
          f" | runs with a pop {sum(any(depth(r) > threshold for r in run) for run in runs)}/{len(runs)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--start', required=True, help='JSON file with "pos": [x, y, z], where each sprint begins')
    parser.add_argument('--arms', default='gpu,hdt', help=f'comma separated, from: {", ".join(ARMS)}')
    parser.add_argument('--runs', type=int, default=6)
    parser.add_argument('--seconds', type=float, default=4.5)
    parser.add_argument('--stride', type=int, default=8)
    parser.add_argument('--drawn', action='store_true', help='measure the skin matrices the GPU draws with; slows the frame a lot')
    parser.add_argument('--bones', default='Butt,Pelvis,Thigh', help='skin vertices whose dominant bone name contains one of these')
    parser.add_argument('--threshold', type=float, default=1.5, help='pop depth when no hdt arm runs')
    parser.add_argument('--rounds', type=int, default=1, help='repeat the arm list, interleaved, to average out drift')
    parser.add_argument('--out', help='write every row to this JSON file')
    arguments = parser.parse_args()
    arms = [arm.strip() for arm in arguments.arms.split(',') if arm.strip()]
    unknown = [arm for arm in arms if arm not in ARMS]
    if unknown:
        raise SystemExit(f'unknown arms {unknown}; choose from {sorted(ARMS)}')
    with open(arguments.start, encoding='utf-8') as handle:
        position = json.load(handle)['pos']

    bench = ingame.Bench(int(os.environ.get('GSP_PORT', '8920')), standalone=True)
    results = {arm: [] for arm in arms}
    try:
        for _ in range(arguments.rounds):
            for arm in arms:
                wanted = {**DEFAULTS, **ARMS[arm]}
                # Some settings only apply when a claim is built, so every arm starts from a rebuilt claim.
                bench.set_settings(**{**wanted, 'Enabled': False})
                time.sleep(4)
                bench.set_settings(**wanted)
                time.sleep(12)
                for _ in range(arguments.runs):
                    for attempt in range(2):
                        try:
                            results[arm].append(sprint(bench, position, arguments.seconds, arguments.stride, arguments.drawn, arguments.bones.split(',')))
                            break
                        except OSError as error:
                            print(f'{arm}: sprint failed ({error}), {"retrying" if attempt == 0 else "skipped"}', flush=True)
                            time.sleep(10)
                print(f'{arm}: {sum(len(run) for run in results[arm])} frames so far', flush=True)
    except OSError as error:
        print(f'stopped early: {error}', flush=True)
    finally:
        for restore in (lambda: bench.command('patchProbe', actorFormID=PLAYER, stop=True), lambda: bench.set_settings(**DEFAULTS),
                        lambda: key(bench, 'up', 'LeftAlt'), lambda: key(bench, 'up', 'W')):
            try:
                restore()
            except Exception:
                pass

    results = {arm: [run for run in runs if run] for arm, runs in results.items()}
    reference = [depth(row) for run in results.get('hdt', []) for row in run]
    threshold = max(reference) if reference else arguments.threshold
    print(f"pop threshold {threshold:.2f} ({'the hdt arm maximum' if reference else '--threshold'})")
    for arm in arms:
        if results[arm]:
            summarize(arm, results[arm], threshold)
        else:
            print(f'{arm:10s} no runs measured')
    if arguments.out:
        with open(arguments.out, 'w', encoding='utf-8') as handle:
            json.dump(results, handle)


if __name__ == '__main__':
    main()
