"""Launch SkyrimSE or SkyrimVR for an in-game test and load a save: python tools/game/launch.py <SE|VR> [--save NAME].

You must already hold that game's exclusive lock under the holder named by GSP_HOLDER (default gpusmpng-vr), see
~/.claude/skyrim-game-coordination.md. The script does not deploy: copy the plugin first (docs in AGENTS.md).
VR turns the SteamVR null driver on and leaves it on once the save is loaded; turn it off with vr-null-toggle.ps1 when done.
It sets the running flag, calls backgroundCompile as soon as devbench answers, loads the save and prints READY.
Quit the game afterwards with the `qqq` console command, never by killing the process.
"""

import argparse
import hashlib
import json
import os
import pathlib
import subprocess
import sys
import time
import urllib.error

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'hair-tune'))
import ingame

STEAM_COMMON = os.environ.get('GSP_STEAM_COMMON', r'E:\SteamLibrary\steamapps\common')
GAMES = {
    'SE': dict(port=8920, dir=os.path.join(STEAM_COMMON, 'Skyrim Special Edition'), loader='skse64_loader.exe', exe='SkyrimSE',
               save='Quicksave0_88D77279_0_507269736F6E6572_WhiterunDragonsreach_000007_20261102042407_1_1'),
    'VR': dict(port=8921, dir=os.path.join(STEAM_COMMON, 'SkyrimVR'), loader='sksevr_loader.exe', exe='SkyrimVR', save='GuardianStonesWater'),
}
HOME = pathlib.Path.home()
COORDINATOR = str(HOME / '.claude' / 'tools' / 'skyrim-game.ps1')
NULL_DRIVER = str(HOME / 'vr-null-toggle.ps1')
HOLDER = os.environ.get('GSP_HOLDER', 'gpusmpng-vr')
LOCK_MINUTES = 20
LOAD_SECONDS = 150  # one load attempt; a save that has not loaded by then is stuck behind something
LOAD_ATTEMPTS = 2


def powershell(command):
    return subprocess.run(['powershell', '-NoProfile', '-Command', command], capture_output=True, text=True)


def wait_for(function, seconds, step=3):
    end = time.time() + seconds
    while time.time() < end:
        try:
            if function():
                return True
        except Exception:
            pass
        time.sleep(step)
    return False


def blockers(bench):
    """The menus devbench says are in the way, as text, so a game idling at the main menu or a modal is reported instead of waited on."""
    try:
        return '; '.join(entry.get('name', '?') + (' (' + entry['hint'] + ')' if entry.get('hint') else '') for entry in bench.post('inspect', {'kind': 'state'}, 5).get('blockedBy', []))
    except Exception as error:
        return f'state unreadable: {error}'


def request_load(bench, wanted, explicit):
    """Asks the game to load a save: the wanted one when it exists, else the newest. A name that is not there is an error only when it was asked for."""
    names = [entry['name'] for entry in bench.post('game', {'action': 'list', 'limit': 500}, 15).get('saves', [])]
    if wanted in names:
        bench.post('game', {'action': 'load', 'name': wanted}, 15)
        return wanted
    if explicit:
        raise SystemExit(f'save {wanted!r} does not exist; {len(names)} saves, newest {names[0] if names else "none"}')
    if not names:
        raise SystemExit('the game has no saves to load')
    print(f'save {wanted!r} does not exist, loading the newest save instead: {names[0]}', flush=True)
    bench.post('game', {'action': 'loadLast'}, 15)
    return names[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('game', choices=sorted(GAMES))
    parser.add_argument('--save', help='save name to load, default the one this project tests with')
    arguments = parser.parse_args()
    config = GAMES[arguments.game]
    bench = ingame.Bench(config['port'], standalone=True)

    def renew():
        powershell(f'& "{COORDINATOR}" renew -Holder {HOLDER} -Game {arguments.game} -Minutes {LOCK_MINUTES} | Out-Null')

    def holds_lock():
        try:
            state = json.loads(powershell(f'& "{COORDINATOR}" status -Game {arguments.game}').stdout)
            state = state.get('games', {}).get(arguments.game, state)
            return state['locks']['exclusive']['holder'] == HOLDER
        except Exception:
            return False

    if not holds_lock():
        raise SystemExit(f'{HOLDER} does not hold the {arguments.game} exclusive lock; acquire it first')
    launched = False
    null_driver_on = False
    keep_running = False
    try:
        renew()
        if arguments.game == 'VR':
            powershell(f'& "{NULL_DRIVER}" on -Force')
            null_driver_on = True
        plugin = pathlib.Path(config['dir']) / 'Data' / 'SKSE' / 'Plugins' / 'GPUSMPPhysicsNG.dll'
        print('plugin', hashlib.sha256(plugin.read_bytes()).hexdigest()[:16] if plugin.exists() else 'NOT DEPLOYED', flush=True)
        powershell(f'Start-Process -FilePath "{config["dir"]}\\{config["loader"]}" -WorkingDirectory "{config["dir"]}"')
        launched = True
        time.sleep(10)
        pid = powershell(f'(Get-Process {config["exe"]}).Id').stdout.strip()
        powershell(f'& "{COORDINATOR}" set-running -Game {arguments.game} -Port {config["port"]} -ProcId {pid} -Exe {config["exe"]}.exe')
        if not wait_for(lambda: bench.post('inspect', {'kind': 'state'}, 5) is not None, 1500):
            raise SystemExit('devbench never answered')
        try:
            bench.post('openshaders.shadercache', {'action': 'backgroundCompile'}, 10)
        except Exception as error:
            print('backgroundCompile skipped:', error)
        for attempt in range(LOAD_ATTEMPTS):
            try:
                loading = request_load(bench, arguments.save or config['save'], bool(arguments.save))
            except urllib.error.HTTPError as error:
                raise SystemExit(f'the game refused the load: HTTP {error.code} {error.read().decode(errors="replace")[:200]}; blocked by: {blockers(bench)}')
            print(f'loading {loading} (attempt {attempt + 1} of {LOAD_ATTEMPTS})', flush=True)
            if wait_for(lambda: bench.post('inspect', {'kind': 'state'}, 5).get('playerLoaded'), LOAD_SECONDS):
                break
            print('not loaded yet, blocked by:', blockers(bench), flush=True)
        else:
            raise SystemExit(f'the save did not load in {LOAD_ATTEMPTS * LOAD_SECONDS} s; blocked by: {blockers(bench)}')
        print('vr =', bench.post('inspect', {'kind': 'state'}, 5).get('vr'), flush=True)
        time.sleep(20)
        renew()
        print('READY', flush=True)
        launched = False
        keep_running = True
    finally:
        if launched:
            try:
                bench.post('console', {'command': 'qqq'}, 10)
            except Exception as error:
                print('qqq', error)
            wait_for(lambda: not powershell(f'(Get-Process {config["exe"]} -ErrorAction SilentlyContinue).Id').stdout.strip(), 120)
        if null_driver_on and not keep_running:
            powershell(f'& "{NULL_DRIVER}" off -Force')
        if not keep_running:
            powershell(f'& "{COORDINATOR}" clear-running -Game {arguments.game} | Out-Null')


if __name__ == '__main__':
    main()
