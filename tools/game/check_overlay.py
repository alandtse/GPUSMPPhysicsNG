"""Turn the debug overlay on and read back its line counts and a screenshot: GSP_PORT (default 8921)."""

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


def lines():
    st = act('invokeQuery', 'SmpPhysics', 'status')
    return st.get('debugLines'), len(st.get('claims', []))


print('before', lines())
print(act('invokeCommand', 'SmpPhysics', 'setSettings', developerMode=True, ShowSkeletonDebug=True, ShowJointDebug=True, ShowColliderDebug=True, DebugMaxActors=3))
time.sleep(4)
print('after', lines())
print('settings', {k: v for k, v in act('invokeQuery', 'SmpPhysics', 'settings').items() if 'Debug' in k})
shot = call('capture', {'action': 'screenshot', 'name': 'gsp_overlay'}, 60)
print('shot', shot)
print('skinProbe registered', 'skinProbe' in [q['name'] for q in act('listQueries', 'SmpPhysics')['queries']])
print('placeCrowdHairAtHand registered', 'placeCrowdHairAtHand' in [c['name'] for c in act('listCommands', 'SmpPhysics')['commands']])
