"""GPU solver cost per pass on one session, for before/after comparisons of shader changes.

Holds the game lock already, game running with the plugin loaded and a save loaded.
Usage: python gpu_pass_cost.py <port> <label> [sizes, e.g. 10,20,30] [armor name part] [cloak name part]
Output: gpu_pass_cost_<vr|se>_<label>.json next to this file, and a table of ms per frame per pass.
The throttles are lifted as in smp_ab.py so every arm simulates every claim.
"""
import json
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 'tools' / 'hair-tune'))
import ingame

PORT = int(sys.argv[1])
LABEL = sys.argv[2]
SIZES = [int(n) for n in sys.argv[3].split(',')] if len(sys.argv) > 3 else [10, 20, 30]
ARMOR = sys.argv[4] if len(sys.argv) > 4 else ''
CLOAK = sys.argv[5] if len(sys.argv) > 5 else ''
GAME = 'SE' if PORT == 8920 else 'VR'
MIXED = [0xA2C8E, 0x13297, 0x13BA3, 0x13BAB, 0x13BB8, 0x13B99, 0x13BB9, 0x13BB0, 0x13B9C, 0x13B9B]
UNTHROTTLED = dict(Enabled=True, GrabStrands=False, MaxClaims=256, MaxSimulatedClaims=256, SimulationDistance=20000.0, MinScreenSizePercent=0.0)
SETTLE = 8
WINDOW = 15
port = ingame.Bench(PORT, standalone=True)
port.set_settings(**UNTHROTTLED)
port.command('reevaluate')
rows = []
for size in SIZES:
    port.command('clearTestCrowd')
    time.sleep(2)
    port.command('spawnTestCrowd', count=size, bases=(MIXED * 3)[:size])
    time.sleep(10 + size * 0.5)
    if ARMOR:
        port.command('setTestCrowdArmor', name=ARMOR)
    if CLOAK:
        port.command('setTestCrowdCloak', name=CLOAK)
    time.sleep(SETTLE + (6 if ARMOR or CLOAK else 0))
    port.command('gpuPassTiming', enabled=True, reset=True)
    time.sleep(WINDOW)
    stats = port.query('gpuPassStats')
    status = port.status()
    passes = stats.get('passes', [])
    solve = next((p for p in passes if p['name'].endswith('RigidSolve')), None)
    islands = port.query('islandSizes')
    row = {'label': LABEL, 'game': GAME, 'crowd': size, 'armor': ARMOR, 'cloak': CLOAK, 'claims': status.get('claimCount', len(status.get('claims', []))),
           'frames': stats.get('frames'), 'substepsPerFrame': round(solve['callsPerFrame'], 3) if solve else None,
           'passes': {p['name']: round(p['msPerFrame'], 4) for p in passes},
           'msPerCall': {p['name']: round(p['msPerFrame'] / p['callsPerFrame'], 4) for p in passes if p['callsPerFrame']},
           'islands': {k: islands.get(k) for k in ('buckets', 'islands', 'constraints', 'total', 'largest', 'mostColors')}}
    port.command('gpuPassTiming', enabled=False)
    print(row, flush=True)
    rows.append(row)
port.command('clearTestCrowd')
json.dump(rows, open(pathlib.Path(__file__).with_name(f'gpu_pass_cost_{GAME.lower()}_{LABEL}.json'), 'w'), indent=1)
print('RESULT DONE', flush=True)
