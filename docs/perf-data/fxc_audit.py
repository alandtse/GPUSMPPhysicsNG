"""Compiles every compute kernel with FXC (cs_5_0, /O3) and prints its compile time, instruction slots, temp
registers, loops and runtime-indexed temp arrays. A kernel whose slot or temp count jumps, or that gains an
indexable temp, is where compile time and register pressure (and so runtime) have gone wrong.
Usage: python fxc_audit.py   (needs the Windows SDK's fxc.exe)
"""
import glob
import pathlib
import re
import subprocess
import tempfile
import time

FXC = sorted(glob.glob(r'C:\Program Files (x86)\Windows Kits\10\bin\*\x64\fxc.exe'))[-1]
ROOT = pathlib.Path(__file__).resolve().parents[2] / 'shaders'
KERNELS = [
    ('ContactCS.hlsl', 'ContactCull'), ('ContactCS.hlsl', 'ContactSpheres'), ('ContactCS.hlsl', 'ContactReduce'),
    ('RigidRuntimeCS.hlsl', 'RigidFrameBegin'), ('RigidRuntimeCS.hlsl', 'RigidWorldExport'),
    ('RigidSolverCS.hlsl', 'RigidPredict'), ('RigidSolverCS.hlsl', 'RigidSetup'), ('RigidSolverCS.hlsl', 'RigidSolveIsland'),
    ('RigidSolverCS.hlsl', 'RigidSolveIslandColored'), ('RigidSolverCS.hlsl', 'RigidIntegrate'),
    ('SurfacePrepCS.hlsl', 'SurfaceBones'), ('SurfacePrepCS.hlsl', 'SurfaceTriangles'), ('SurfacePrepCS.hlsl', 'SurfaceLeaves'),
    ('SurfacePrepCS.hlsl', 'SurfacePose'), ('SurfacePrepCS.hlsl', 'SurfaceRefit'),
    ('SkinPaletteBatchCS.hlsl', 'main'), ('SkinPaletteCS.hlsl', 'main'),
]

print('%-26s %-24s %7s %6s %6s %6s  %s' % ('file', 'kernel', 'sec', 'slots', 'temps', 'loops', 'indexable temps'))
with tempfile.TemporaryDirectory() as out:
    for name, entry in KERNELS:
        asm = pathlib.Path(out) / ('%s_%s.asm' % (name, entry))
        start = time.time()
        proc = subprocess.run([FXC, '/nologo', '/T', 'cs_5_0', '/E', entry, '/O3', '/I', str(ROOT), '/I', str(ROOT / 'Common'),
                               '/I', str(ROOT / 'SmpPhysics'), '/Fc', str(asm), str(ROOT / 'SmpPhysics' / name)], capture_output=True, text=True)
        seconds = time.time() - start
        if not asm.exists():
            print('%-26s %-24s %7.1f FAILED %s' % (name, entry, seconds, (proc.stdout + proc.stderr).strip().splitlines()[:1]))
            continue
        text = asm.read_text(errors='replace')
        slots = re.search(r'Approximately (\d+) instruction slots used', text)
        temps = re.search(r'dcl_temps (\d+)', text)
        indexable = ['%sx%s' % (m.group(2), m.group(1)) for m in re.finditer(r'dcl_indexableTemp x\d+\[(\d+)\],\s*(\d+)', text)]
        loops = len(re.findall(r'^\s*loop\b', text, re.M))
        print('%-26s %-24s %7.1f %6s %6s %6d  %s' % (name, entry, seconds, slots.group(1) if slots else '?', temps.group(1) if temps else '?', loops, ' '.join(indexable)))
