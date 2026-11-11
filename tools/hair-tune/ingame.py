"""Shared helpers for the GPU SMP physics in-game tests (grab_e2e.py, pose_e2e.py).

They talk to a running game through the devbench server, so the game must already be up with the devbench
server answering, a save loaded, and you holding its lock. Nothing here launches or quits the game.
"""

import json
import os
import sys
import time
import urllib.error
import urllib.request

GRIP_BUTTON = 1 << 2  # OpenVR k_EButton_Grip in a controller's pressed and touched masks
EYE_HEIGHT = 1.6  # tracking-space height of the headset, in metres


CROWD_NAMES = {'spawnTestCrowd', 'clearTestCrowd', 'refreshTestCrowd', 'turnTestCrowd', 'stripTestCrowd', 'cycleTestCrowdArmor',
               'playTestCrowdAnimation', 'setTestCrowdLoop', 'testCrowdAnimations', 'testCrowdOutfit', 'testCrowdReport',
               'setTestCrowdCloak', 'testCrowdCloaks', 'setTestCrowdArmor'}


class Bench:
    """A devbench client for one game."""

    def __init__(self, port, standalone=None):
        self.port = port
        self.standalone = os.environ.get('GPUSMP_STANDALONE') == '1' if standalone is None else standalone
        self.actions_tool = 'gpusmp.actions' if self.standalone else 'openshaders.actions'

    def post(self, tool, body, timeout=30):
        request = urllib.request.Request(f'http://127.0.0.1:{self.port}/api/tool/{tool}', json.dumps(body).encode(), {'Content-Type': 'application/json'})
        return json.load(urllib.request.urlopen(request, timeout=timeout))

    @staticmethod
    def owner(name):
        """The devbench owner of a command: the test crowd has its own tool owner, TestCrowd, apart from the hair colour commands."""
        return 'TestCrowd' if name in CROWD_NAMES else 'SmpPhysics'

    def command(self, name, /, **args):
        return self.post(self.actions_tool, {'action': 'invokeCommand', 'shortName': self.owner(name), 'name': name, 'args': args})

    def query(self, name, **args):
        reply = self.post(self.actions_tool, {'action': 'invokeQuery', 'shortName': self.owner(name), 'name': name, 'args': args})
        return reply.get('result', reply) if isinstance(reply, dict) else reply

    def status(self):
        return self.query('status')

    def place_hair_at_hand(self, hand, index=0, attempts=8, away=0.0):
        """Moves a test character's hair to a hand, retrying while its claim is still coming up; true when placed."""
        for _ in range(attempts):
            self.command('placeCrowdHairAtHand', hand=hand, index=index, away=away)
            time.sleep(1.0)
            if str(self.query('testCrowdReport').get('report', '')).startswith('Placed'):
                return True
        return False

    def set_settings(self, **settings):
        if self.standalone:
            return self.command('setSettings', **settings)
        return self.post('openshaders.feature', {'action': 'set', 'shortName': 'SmpPhysics', 'settings': settings}, 15)


class Checks:
    """Records named checks, prints each as it is made, and turns them into an exit code."""

    def __init__(self):
        self.failed = []
        self.count = 0

    def check(self, name, ok, detail=''):
        self.count += 1
        print(f"{'PASS' if ok else 'FAIL'}  {name}{('  ' + str(detail)) if detail != '' else ''}", flush=True)
        if not ok:
            self.failed.append(name)
        return ok

    def finish(self):
        print(f'{self.count - len(self.failed)} of {self.count} checks passed', flush=True)
        if self.failed:
            print('failed: ' + ', '.join(self.failed), flush=True)
        return 1 if self.failed else 0


def wait_until(function, seconds, step=0.5):
    """Polls function until it returns something truthy, and returns that, or None after the time is up."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        try:
            value = function()
            if value:
                return value
        except Exception:
            pass
        time.sleep(step)
    return None


def pose(x, y, z):
    """An OpenVR device-to-absolute 3x4 row-major matrix: no rotation, at the given tracking-space position."""
    return [1.0, 0.0, 0.0, x, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, z]


def _device(index, role, position, pressed, packet):
    squeeze = 1.0 if pressed else 0.0
    mask = GRIP_BUTTON if pressed else 0
    return {'angularVelocity': [0, 0, 0], 'available': True, 'connected': True, 'deviceClass': 2, 'index': index, 'matrix': pose(*position), 'role': role,
            'trackingResult': 200, 'valid': True, 'velocity': [0, 0, 0],
            'controller': {'axes': [[0, 0], [0, 0], [squeeze, 0], [0, 0], [0, 0]], 'packetNumber': packet, 'pressed': mask, 'touched': mask}}


def tracked_frames(duration_ms, left, right, step_ms=50):
    """Frames for a vrTrackedSet sequence.

    left and right are functions of the time in milliseconds returning ((x, y, z), gripPressed) for that controller,
    in tracking space. The headset stays at standing eye height.
    """
    frames = []
    packet = 1
    for t in range(0, duration_ms, step_ms):
        left_position, left_pressed = left(t)
        right_position, right_pressed = right(t)
        frames.append({'tMs': t, 'originCode': 1,
                       'hmd': {'angularVelocity': [0, 0, 0], 'available': True, 'connected': True, 'deviceClass': 1, 'index': 0, 'matrix': pose(0.0, EYE_HEIGHT, 0.0),
                               'role': 'hmd', 'trackingResult': 200, 'valid': True, 'velocity': [0, 0, 0]},
                       'left': _device(1, 'left', left_position, left_pressed, packet),
                       'right': _device(2, 'right', right_position, right_pressed, packet)})
        packet += 1
    return frames


def start_tracked_sequence(bench, frames, owner):
    """Starts a vrTrackedSet sequence and returns its control token."""
    reply = bench.post('input', {'action': 'sequence', 'device': 'vrTrackedSet', 'frames': frames, 'owner': owner})
    if not reply.get('queued'):
        sys.exit(f'the tracked sequence was not queued: {reply}')
    return reply['controlToken']


def stop_tracked_sequence(bench, token):
    try:
        bench.post('input', {'action': 'stop', 'device': 'vrTrackedSet', 'controlToken': token})
    except urllib.error.HTTPError as error:
        if error.code != 409:  # 409: the sequence already ended
            print(f'stopping the tracked sequence failed: {error}', flush=True)
