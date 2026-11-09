"""Batch driver for the hair solver benchmark: swaps wigs on the player and records SMP traces.

    python tools/hair-tune/bench.py pin                 # before launching the game: pin hdtSMP64's config for the benchmark
    python tools/hair-tune/bench.py unpin               # restore it afterwards
    python tools/hair-tune/bench.py enum  [--first 0xFE003800 --last 0xFE004000]   # list head parts by name
    python tools/hair-tune/bench.py map   --wigs Esmerald,Lena [--out rigmap.json] # equip candidates, learn which rig each loads
    python tools/hair-tune/bench.py capture --wigs Esmerald,Lena --out traces [--motions turn,sweep,shake]

Everything except pin/unpin needs the game running with the devbench server and you holding its lock.
`map` writes rig file -> head part form; `capture` swaps each wig in and records one trace per motion.
"""

import argparse
import json
import re
import shutil
import sys
import time
from pathlib import Path

import capture as cap

GAME = Path('E:/SteamLibrary/steamapps/common/Skyrim Special Edition')
CONFIG = GAME / 'Data/SKSE/Plugins/hdtSkinnedMeshConfigs/configs.json'
BACKUP = Path('F:/tmp/backups/configs.json.pre-bench')
RIG_ROOTS = [GAME / 'Data/meshes']


def pin():
    BACKUP.parent.mkdir(parents=True, exist_ok=True)
    if not BACKUP.exists():
        shutil.copyfile(CONFIG, BACKUP)
    text = CONFIG.read_text(encoding='utf-8-sig')
    config = json.loads(text)
    config['smp'].update({'autoAdjustMaxSkeletons': False, 'maximumActiveSkeletons': 40, 'minCullingDistance': 100000.0,
                          'clampRotations': False})
    config['wind']['enabled'] = False
    CONFIG.write_text(json.dumps(config, indent=1), encoding='utf-8')
    print('pinned', CONFIG, '(backup at', BACKUP, ')')


def unpin():
    if BACKUP.exists():
        shutil.copyfile(BACKUP, CONFIG)
        BACKUP.unlink()
        print('restored', CONFIG)
    else:
        print('no backup, nothing to restore')


RIG_BACKUPS = Path('F:/tmp/backups/rigs')


def strip_collision(rig, restore=False):
    """Removes (or restores) a rig's collision shapes so hdtSMP64 simulates it the way our collision-off solver does.

    Vortex deploys by hard link, so the staging copy changes with the Data copy; always restore afterwards.
    """
    rig = Path(rig)
    backup = RIG_BACKUPS / rig.name
    if restore:
        if backup.exists():
            shutil.copyfile(backup, rig)
            backup.unlink()
            print('restored', rig.name)
        return
    RIG_BACKUPS.mkdir(parents=True, exist_ok=True)
    if not backup.exists():
        shutil.copyfile(rig, backup)
    text = backup.read_text(encoding='utf-8', errors='replace')
    for tag in ('per-triangle-shape', 'per-vertex-shape'):
        text = re.sub(rf'<{tag}.*?</{tag}>', '', text, flags=re.S)
    rig.write_text(text, encoding='utf-8')
    print('stripped collision from', rig.name)


def set_smp_config(settings):
    """Edits hdtSMP64's configs.json (section.key=value) and reloads it with the console's smp reset."""
    config = json.loads(CONFIG.read_text(encoding='utf-8-sig'))
    for item in settings:
        path, value = item.split('=')
        section, key = path.split('.')
        config[section][key] = json.loads(value)
    CONFIG.write_text(json.dumps(config, indent=1), encoding='utf-8')
    cap.post('console', {'command': 'smp reset'}, 15)
    time.sleep(4)
    print('applied', settings)


def write_simple_rig(rig, stiffness=1000.0, damping=76.0):
    """Replaces a wig's rig with a linear one of known parameters: no collision, no body damping, wide limits.

    The spring is well under the stability cap and the limits are never reached, so the response is a damped
    oscillator whose frequency and damping follow from the numbers below (inertia 16).
    """
    rig = Path(rig)
    backup = RIG_BACKUPS / rig.name
    RIG_BACKUPS.mkdir(parents=True, exist_ok=True)
    if not backup.exists():
        shutil.copyfile(rig, backup)
    text = backup.read_text(encoding='utf-8', errors='replace')
    for tag in ('per-triangle-shape', 'per-vertex-shape'):
        text = re.sub(rf'<{tag}.*?</{tag}>', '', text, flags=re.S)
    text = re.sub(r'<linearDamping>[^<]*</linearDamping>', '<linearDamping>0</linearDamping>', text, count=1)
    text = re.sub(r'<angularDamping>[^<]*</angularDamping>', '<angularDamping>0</angularDamping>', text, count=1)
    text = re.sub(r'<gravity-factor>[^<]*</gravity-factor>', '<gravity-factor>1</gravity-factor>', text, count=1)
    block = f"""<generic-constraint-default>
		<frameInB>
			<basis x="0" y="0" z="0" w="1" />
			<origin x="0" y="0" z="0" />
		</frameInB>
		<useLinearReferenceFrameA>false</useLinearReferenceFrameA>
		<linearLowerLimit x="0" y="0" z="0" />
		<linearUpperLimit x="0" y="0" z="0" />
		<angularLowerLimit x="-1.5" y="-1.5" z="-1.5" />
		<angularUpperLimit x="1.5" y="1.5" z="1.5" />
		<linearStiffness x="0" y="0" z="0" />
		<angularStiffness x="{stiffness}" y="{stiffness}" z="{stiffness}" />
		<linearDamping x="0" y="0" z="0" />
		<angularDamping x="{damping}" y="{damping}" z="{damping}" />
		<linearEquilibrium x="0" y="0" z="0" />
		<angularEquilibrium x="0" y="0" z="0" />
	</generic-constraint-default>"""
    text = re.sub(r'<generic-constraint-default>.*?</generic-constraint-default>', lambda match: block, text, count=1, flags=re.S)
    rig.write_text(text, encoding='utf-8')
    print('wrote the simple rig over', rig.name)


def papyrus(script, function, form=None, args=None, timeout=15000):
    body = {'action': 'call', 'script': script, 'function': function, 'args': args or [], 'timeoutMs': timeout}
    if form:
        body['self'] = {'form': form}
    try:
        return cap.post('papyrus', body, 40)
    except Exception as error:
        return {'error': str(error)}


def form_name(form_id):
    reply = papyrus('Form', 'GetName', hex(form_id), None, 3000)
    return reply.get('returned') if isinstance(reply, dict) else None


def enum(first, last, out):
    names = {}
    for form_id in range(first, last + 1):
        name = form_name(form_id)
        if name:
            names[hex(form_id)] = name
    Path(out).write_text(json.dumps(names, indent=1))
    print(f'{len(names)} named forms between {hex(first)} and {hex(last)} -> {out}')
    return names


def normalise(text):
    return re.sub(r'[^a-z0-9]', '', text.lower())


def resolve(file):
    path = Path(file.replace(chr(92), '/'))
    return path if path.is_absolute() else GAME / 'Data' / path


def equip(form):
    papyrus('Actor', 'ChangeHeadPart', '0x14', [{'form': form}])
    papyrus('Actor', 'QueueNiNodeUpdate', '0x14')
    time.sleep(9)
    status = cap.query('status')
    files = [c['file'] for c in status.get('claims', [])] + [d['file'] for d in status.get('declined', [])]
    return [resolve(f) for f in files]


def find_rig(basename):
    for root in RIG_ROOTS:
        for path in root.rglob(f'{basename}.xml'):
            return path
    return None


def do_map(wigs, headparts_file, out):
    names = json.loads(Path(headparts_file).read_text())
    mapping = json.loads(Path(out).read_text()) if Path(out).exists() else {}
    for wig in wigs:
        token = normalise(wig)
        candidates = [form for form, name in names.items() if token in normalise(name)]
        print(wig, 'candidates', [(f, names[f]) for f in candidates])
        for form in candidates:
            files = equip(form)
            for path in files:
                if normalise(path.stem) == token or token in normalise(path.stem):
                    mapping[wig] = {'form': form, 'rig': str(path).replace('\\', '/'), 'name': names[form]}
                    print('  ', wig, '->', form, names[form], path.name)
                    break
            if wig in mapping:
                break
        else:
            print('  no head part loaded a rig matching', wig)
    Path(out).write_text(json.dumps(mapping, indent=1))


def clear_head_slots():
    """hdtSMP64 marks the head inactive while armor sits in a hair or long hair slot, so clear those slots."""
    for slot in (30, 31, 41, 43, 130, 131):
        papyrus('Actor', 'UnequipItemSlot', '0x14', [slot])


def smp_wig_active():
    """True when hdtSMP64 reports an active physics system on the player's active skeleton."""
    cap.post('console', {'command': 'smp detail', 'capture': True}, 15)
    time.sleep(2)
    lines = cap.post('console', {'action': 'read'}, 15).get('lines', [])
    inside = False
    active = 0
    for line in lines:
        if 'skeleton' in line:
            inside = 'Is player character' in line
        elif inside and 'has active physics system' in line:
            active += 1
    return active


def do_capture(wigs, mapping_file, out_dir, motions, frames):
    cap.post('camera', {'action': 'setPov', 'pov': 'third'})  # hdtSMP64 skips first person view physics
    time.sleep(2)
    mapping = json.loads(Path(mapping_file).read_text())
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for wig in wigs:
        entry = mapping.get(wig)
        if not entry:
            print('skip', wig, '(not in the map; run map first)')
            continue
        clear_head_slots()
        files = equip(entry['form'])
        active = smp_wig_active()
        print(wig, 'active SMP physics systems on the player:', active)
        if not active:
            print('skip', wig, '(hdtSMP64 is not simulating it)')
            continue
        rig = entry['rig']
        info = cap.rig_info(str(cap.EXE), rig)
        dynamic = [b['name'] for b in info['bones'] if not b['kinematic']]
        if not dynamic:
            print('skip', wig, '(no dynamic bones)')
            continue
        prefix = 'hdtSSEPhysics'
        print(wig, 'prefix', repr(prefix), 'rig', Path(rig).name, 'dynamic bones', len(dynamic))
        for motion in motions:
            path = out_dir / f'{wig}_{motion}.json'
            frames_saved, nodes = cap.capture(rig, prefix, str(path), motion, frames)
            print(f'  {motion}: {frames_saved} frames, {nodes} nodes -> {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=['pin', 'unpin', 'enum', 'map', 'capture', 'nocollide', 'restorerigs', 'smpset', 'simplerig'])
    parser.add_argument('--wigs', default='')
    parser.add_argument('--out', default='')
    parser.add_argument('--set', action='append', default=[], help='smpset: section.key=value, e.g. solver.numIterations=64')
    parser.add_argument('--first', default='0xFE003800')
    parser.add_argument('--last', default='0xFE004000')
    parser.add_argument('--headparts', default='F:/tmp/hairbench/headparts.json')
    parser.add_argument('--rigmap', default='F:/tmp/hairbench/rigmap.json')
    parser.add_argument('--motions', default='turn,sweep,shake')
    parser.add_argument('--frames', type=int, default=900)
    parser.add_argument('--port', type=int, default=cap.PORT)
    args = parser.parse_args()
    cap.PORT = args.port
    wigs = [w for w in args.wigs.split(',') if w]
    Path('F:/tmp/hairbench').mkdir(parents=True, exist_ok=True)
    if args.command == 'pin':
        pin()
    elif args.command == 'unpin':
        unpin()
    elif args.command == 'enum':
        enum(int(args.first, 16), int(args.last, 16), args.out or args.headparts)
    elif args.command == 'map':
        do_map(wigs, args.headparts, args.out or args.rigmap)
    elif args.command == 'smpset':
        set_smp_config(args.set)
    elif args.command == 'simplerig':
        mapping = json.loads(Path(args.rigmap).read_text())
        for wig in wigs:
            write_simple_rig(mapping[wig]['rig'])
    elif args.command in ('nocollide', 'restorerigs'):
        mapping = json.loads(Path(args.rigmap).read_text())
        for wig in wigs:
            strip_collision(mapping[wig]['rig'], restore=args.command == 'restorerigs')
    elif args.command == 'capture':
        do_capture(wigs, args.rigmap, args.out or 'F:/tmp/hairbench/traces', args.motions.split(','), args.frames)


if __name__ == '__main__':
    sys.exit(main())
