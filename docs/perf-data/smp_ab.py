"""A/B on one session: hdtSMP64 only vs standalone GPUSMPPhysicsNG (the in-tree Open Shaders arm is gone).

Holds the game lock already, game running with both plugins loaded and a save loaded.
Usage: python smp_ab.py [port] [sizes, e.g. 0,10,20]   Output: smp_ab_<vr|se>_results.json next to this file and a table.
"""
import json
import os
import pathlib
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 'tools' / 'hair-tune'))
import ingame

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8921
GAME = 'SE' if PORT == 8920 else 'VR'
BUDGET_MS = 16.67 if GAME == 'SE' else 11.11
GAME_DIR = {'SE': 'Skyrim Special Edition', 'VR': 'SkyrimVR'}[GAME]
MIXED = [0xA2C8E, 0x13297, 0x13BA3, 0x13BAB, 0x13BB8, 0x13B99, 0x13BB9, 0x13BB0, 0x13B9C, 0x13B9B]
SIZES = [int(n) for n in sys.argv[2].split(',')] if len(sys.argv) > 2 else [0, 5, 10, 20, 30]
SETTLE = 6
WINDOW = 15
port = ingame.Bench(PORT, standalone=True)


def state():
    for attempt in range(60):
        try:
            return _state()
        except urllib.error.HTTPError as error:
            if error.code != 504:
                raise
            time.sleep(3)
    raise RuntimeError('devbench kept answering 504')


def _state():
    request = urllib.request.Request(f'http://127.0.0.1:{PORT}/api/tool/inspect', json.dumps({'kind': 'state'}).encode(), {'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(request, timeout=10).read())


def fps(seconds):
    a = state()['frame']
    t = time.time()
    time.sleep(seconds)
    b = state()['frame']
    return (b - a) / (time.time() - t)


UNTHROTTLED = dict(GrabStrands=False, MaxClaims=256, MaxSimulatedClaims=256, SimulationDistance=20000.0, MinScreenSizePercent=0.0)
HDT_CONFIG = f'{os.environ.get("GSP_STEAM_COMMON", "E:/SteamLibrary/steamapps/common")}/{GAME_DIR}/Data/SKSE/Plugins/hdtSkinnedMeshConfigs/configs.json'


def check_hdt_unthrottled():
    cfg = json.load(open(HDT_CONFIG, encoding='utf-8-sig'))['smp']
    ok = (not cfg['autoAdjustMaxSkeletons'] and cfg['maximumActiveSkeletons'] >= 100 and cfg['budgetMs'] >= 100 and cfg['minCullingDistance'] >= 20000 and cfg['minScreenSizePercent'] == 0)
    print('hdtSMP64 throttling config', {k: cfg[k] for k in ('autoAdjustMaxSkeletons', 'maximumActiveSkeletons', 'budgetMs', 'minCullingDistance', 'minScreenSizePercent')}, flush=True)
    if not ok:
        raise SystemExit('hdtSMP64 is still throttled; edit configs.json (backed up) and restart the game first')


def arm(name):
    port.set_settings(Enabled=(name == 'port'), **UNTHROTTLED)
    port.command('reevaluate')


def measure(name, size):
    arm(name)
    time.sleep(SETTLE)
    if name == 'port':
        port.command('gpuPassTiming', enabled=True, reset=True)
    rate = fps(WINDOW)
    status = port.status() if name == 'port' else {}
    row = {'arm': name, 'crowd': size, 'fps': round(rate, 2), 'frameMs': round(1000.0 / rate, 3),
           'claims': status.get('claimCount', len(status.get('claims', []))), 'stepUs': status.get('totalStepMicroseconds')}
    if name == 'port':
        stats = port.query('gpuPassStats')
        row['gpuPasses'] = {p['name']: round(p['msPerFrame'], 4) for p in stats.get('passes', [])}
        row['gpuFrames'] = stats.get('frames')
        port.command('gpuPassTiming', enabled=False)
    print(row, flush=True)
    return row


check_hdt_unthrottled()
st = state()
port.command('spawnTestCrowd', count=5, bases=MIXED[:5])
time.sleep(15)
for warm in ('port', 'hdt'):
    arm(warm)
    for attempt in range(40):
        if fps(5) > 10:
            break
    time.sleep(5)
port.command('clearTestCrowd')
print('warm-up done', flush=True)

print('state', {k: st.get(k) for k in ('vr', 'frameGenActive', 'frame')}, flush=True)
results = []
for size in SIZES:
    port.command('clearTestCrowd')
    arm('hdt')
    time.sleep(2)
    if size:
        port.command('spawnTestCrowd', count=size, bases=(MIXED * 3)[:size])
        time.sleep(10 + size * 0.5)
    for name in ('hdt', 'port', 'hdt'):
        results.append(measure(name, size))
arm('hdt')
port.command('clearTestCrowd')
json.dump(results, open(pathlib.Path(__file__).with_name(f'smp_ab_{GAME.lower()}_results_{time.strftime("%Y-%m-%d")}_{"_".join(map(str, SIZES))}.json'), 'w'), indent=1)
print('RESULT DONE', flush=True)
