"""本次整机验收记录；只控制当前localhost虚拟机械臂界面，不是产品入口。"""
import json
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
records = {}

def state():
    with urllib.request.urlopen('http://127.0.0.1:8767/api/state', timeout=3) as response:
        return json.load(response)

def send(command):
    request = urllib.request.Request('http://127.0.0.1:8767/api/command',
        json.dumps({'command': command}).encode(), {'Content-Type': 'application/json'})
    urllib.request.urlopen(request, timeout=3).close()

def wait(predicate, seconds=30):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        value = state()
        if predicate(value):
            return value
        time.sleep(.05)
    raise AssertionError(state())

def fresh():
    send('new')
    time.sleep(.15)
    return wait(lambda s: s['phase'] == 'waiting' and not s['history'] and len(s['legal']) == 44)

try:
    initial = fresh()
    send('move e0e9')
    records['illegal'] = wait(lambda s: '不合法' in s['message'])
    assert records['illegal']['board'] == initial['board'] and not records['illegal']['history']

    send('move e3e4')
    wait(lambda s: s['phase'] == 'thinking' and s['history'] == 'e3e4')
    start = time.monotonic()
    send('pause')
    records['pause_thinking'] = wait(lambda s: s['phase'] == 'paused')
    records['pause_thinking_seconds'] = time.monotonic() - start
    assert records['pause_thinking']['history'] == 'e3e4'

    fresh()
    send('scan')
    wait(lambda s: s['phase'] == 'scanning')
    start = time.monotonic()
    send('pause')
    records['pause_camera'] = wait(lambda s: s['phase'] == 'paused')
    records['pause_camera_seconds'] = time.monotonic() - start
    assert not records['pause_camera']['history']

    fresh()
    send('move e3e4')
    wait(lambda s: s['phase'] == 'moving' and s['carried'] != '.')
    send('pause')
    paused = wait(lambda s: s['phase'] == 'paused')
    time.sleep(.4)
    records['pause_carrying'] = state()
    assert paused['history'] == 'e3e4'
    assert records['pause_carrying']['world'] == paused['world']
    assert records['pause_carrying']['tcp'] == paused['tcp']
    assert records['pause_carrying']['carried'] != '.' and records['pause_carrying']['vacuum'] == 1

    fresh()
    send('resign')
    records['resign'] = wait(lambda s: s['phase'] == 'finished')
    assert records['resign']['result'] == 'black_won'
    fresh()
    send('draw')
    records['draw'] = wait(lambda s: s['phase'] == 'finished')
    assert records['draw']['result'] == 'draw'

    fresh()
    send('move e3e4')
    wait(lambda s: s['phase'] == 'waiting' and len(s['history'].split()) == 2)
    send('demo 3')
    samples = []
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        value = state()
        samples.append(value)
        if value['phase'] == 'waiting' and len(value['history'].split()) == 8:
            break
        assert value['phase'] not in ('error', 'paused')
        time.sleep(.05)
    assert len(value['history'].split()) == 8
    assert value['world'] == value['board'] and value['carried'] == '.' and not value['vacuum']
    assert any(s['carried'] != '.' and s['vacuum'] for s in samples)
    records['continuous'] = value
    records['movement_samples'] = samples
    records['passed'] = True
finally:
    (ROOT / 'live-results.json').write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({k:v for k,v in records.items() if k.endswith('seconds') or k == 'passed'}))
