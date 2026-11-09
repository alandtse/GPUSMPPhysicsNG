"""Query the standalone plugin's devbench actions on a running game: GSP_PORT (default 8921) selects SE or VR."""

import json, time, urllib.request

import os
PORT = int(os.environ.get('GSP_PORT', '8921'))


def call(tool, body, timeout=40):
    req = urllib.request.Request(f'http://127.0.0.1:{PORT}/api/tool/{tool}', data=json.dumps(body).encode(), headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(req, timeout=timeout).read())


def act(action, shortName=None, name=None, **args):
    body = {'action': action}
    if shortName:
        body['shortName'] = shortName
    if name:
        body['name'] = name
    if args:
        body['args'] = args
    return call('gpusmp.actions', body)


def wait(fn, seconds, step=1.0):
    end = time.time() + seconds
    while time.time() < end:
        try:
            v = fn()
            if v:
                return v
        except Exception:
            pass
        time.sleep(step)
    return None


print('owners', act('listCommands'))
print('SmpPhysics queries', [q['name'] for q in act('listQueries', 'SmpPhysics')['queries']])
print('TestCrowd commands', [c['name'] for c in act('listCommands', 'TestCrowd')['commands']])
st = act('invokeQuery', 'SmpPhysics', 'status')
print('status claims', len(st.get('claims', [])), {k: st[k] for k in list(st)[:6]})
print('spawn', act('invokeCommand', 'TestCrowd', 'spawnTestCrowd', count=6))
ok = wait(lambda: len(act('invokeQuery', 'TestCrowd', 'testCrowdOutfit')) == 6, 60)
print('crowd stands', bool(ok), act('invokeQuery', 'TestCrowd', 'testCrowdReport'))
time.sleep(8)
print('reevaluate', act('invokeCommand', 'SmpPhysics', 'reevaluate'))
time.sleep(8)
st = act('invokeQuery', 'SmpPhysics', 'status')
print('claims after crowd', len(st.get('claims', [])))
print('hair colours', len(act('invokeQuery', 'TestCrowd', 'testCrowdHairColors')) if False else 'skipped')
print('set colour', act('invokeCommand', 'SmpPhysics', 'setTestCrowdHairColor', color='Black'))
time.sleep(6)
print('report', act('invokeQuery', 'TestCrowd', 'testCrowdReport'))
