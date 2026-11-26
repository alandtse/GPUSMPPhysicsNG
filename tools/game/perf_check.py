"""Record this build's frame time and GPU cost per solver step against hdtSMP64, in the settings the game has.

Run it on a running game (GSP_PORT, default 8920) with a save loaded, under a perf-test lock, before a commit that
touches the solver, its shaders or its settings. It appends one line to docs/perf-data/perf_log.jsonl with the commit,
so a regression can be traced to the commit that brought it. No throttle is lifted: hdtSMP64 and the plugin both run
as the user has them configured.

    python tools/game/perf_check.py --start F:/Temp/stairs_top.json --label "what changed"

--start is a JSON file {"pos": [x, y, z], "angle": degrees} to stand at. Frame time is wall time over the engine's
frame counter. The plugin's frame time has more than one steady state (a longer frame takes more solver steps, which
cost GPU time), so the steps per frame are recorded with it and the GPU cost is given per step.
"""

import argparse
import json
import os
import pathlib
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'hair-tune'))
import ingame  # noqa: E402

PLAYER = 0x14
REPO = pathlib.Path(__file__).resolve().parents[2]
LOG = REPO / 'docs' / 'perf-data' / 'perf_log.jsonl'
GAME_MENUS = ('HUD Menu', 'Cursor Menu')
PASSES = ('RigidContact', 'ContactPasses', 'ContactReduce', 'ContactSpheres', 'ClothVertices', 'ClothTriangles', 'ClothFold', 'SurfacePrep', 'RigidSolve')


def git(*arguments):
    return subprocess.run(['git', *arguments], cwd=REPO, capture_output=True, text=True).stdout.strip()


def frame(bench):
    return bench.post('inspect', {'kind': 'state'}, 15)['frame']


def frame_ms(bench, seconds):
    first, began = frame(bench), time.time()
    time.sleep(seconds)
    last, ended = frame(bench), time.time()
    return 1000.0 * (ended - began) / max(1, last - first)


def blocking_menus(bench):
    return [name for name in bench.post('menu', {'action': 'list'}, 15).get('openMenus', []) if name not in GAME_MENUS]


def key(bench, action, name):
    body = {'action': action, 'device': 'keyboard', 'key': name}
    if action == 'down':
        body['maxHoldMs'] = 60000
    bench.post('input', body, 15)


def papyrus(bench, function, arguments):
    bench.post('papyrus', {'action': 'call', 'script': 'ObjectReference', 'function': function, 'self': {'form': hex(PLAYER)}, 'args': arguments}, 15)


def measure(bench, start, seconds):
    """Frame time standing and running from the start spot; None for a run a menu interrupted."""
    for name in blocking_menus(bench):
        bench.post('menu', {'action': 'close', 'name': name}, 15)
        time.sleep(1.5)
    key(bench, 'up', 'W')
    papyrus(bench, 'SetPosition', start['pos'])
    papyrus(bench, 'SetAngle', [0.0, 0.0, start.get('angle', 0.0)])
    time.sleep(3)
    standing = frame_ms(bench, seconds)
    key(bench, 'down', 'W')
    try:
        time.sleep(0.6)
        running = frame_ms(bench, seconds * 0.7)
    finally:
        key(bench, 'up', 'W')
    time.sleep(0.5)
    return None if blocking_menus(bench) else (standing, running)


def pass_costs(bench, seconds):
    """GPU milliseconds per solver step of each pass, and the steps taken per frame while they were timed."""
    bench.command('gpuPassTiming', enabled=True, reset=True)
    time.sleep(seconds)
    stats = bench.query('gpuPassStats')
    bench.command('gpuPassTiming', enabled=False)
    passes = {entry['name'].split('::')[-1]: entry for entry in stats.get('passes', [])}
    per_step = {name: round(passes[name]['msPerFrame'] / passes[name]['callsPerFrame'], 3) for name in PASSES if name in passes and passes[name]['callsPerFrame']}
    steps = passes.get('RigidSolve', {}).get('callsPerFrame')
    return per_step, steps


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--start', required=True, help='JSON file with "pos": [x, y, z] and "angle"')
    parser.add_argument('--label', default='', help='what this build changes')
    parser.add_argument('--rounds', type=int, default=2)
    parser.add_argument('--seconds', type=float, default=5.0)
    parser.add_argument('--scene', default='', help='a name for the save and place, so lines are compared like with like')
    parser.add_argument('--no-log', action='store_true', help='print the line without appending it')
    arguments = parser.parse_args()
    with open(arguments.start, encoding='utf-8') as handle:
        start = json.load(handle)

    bench = ingame.Bench(int(os.environ.get('GSP_PORT', '8920')), standalone=True)
    arms = {'ours': [], 'hdt': []}
    claims = None
    work = None
    try:
        for _ in range(arguments.rounds):
            for arm, enabled, settle in (('ours', True, 12), ('hdt', False, 10)):
                bench.set_settings(Enabled=enabled)
                time.sleep(settle)
                result = measure(bench, start, arguments.seconds)
                if result:
                    arms[arm].append(result)
                if enabled:
                    status = bench.status()
                    claims, work = status.get('gpuClaimCount'), status.get('gpuContactWork')
        bench.set_settings(Enabled=True)
        time.sleep(12)
        per_step, steps = pass_costs(bench, 6)
    finally:
        key(bench, 'up', 'W')
        bench.set_settings(Enabled=True)

    def mean(arm, index):
        return round(sum(run[index] for run in arms[arm]) / len(arms[arm]), 2) if arms[arm] else None

    line = {
        'date': time.strftime('%Y-%m-%d %H:%M'),
        'commit': git('rev-parse', '--short', 'HEAD'),
        'dirty': bool(git('status', '--short')),
        'label': arguments.label,
        'scene': arguments.scene,
        'gpuClaims': claims,
        'gpuContactWork': work,
        'ours': {'standing': mean('ours', 0), 'running': mean('ours', 1)},
        'hdtSMP64': {'standing': mean('hdt', 0), 'running': mean('hdt', 1)},
        'stepsPerFrameWhileTimed': steps,
        'gpuMsPerStep': per_step,
        'gpuMsPerStepTotal': round(per_step.get('RigidContact', 0.0) + per_step.get('RigidSolve', 0.0), 3),
    }
    print(json.dumps(line))
    if not arguments.no_log:
        with open(LOG, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(line) + '\n')


if __name__ == '__main__':
    main()
