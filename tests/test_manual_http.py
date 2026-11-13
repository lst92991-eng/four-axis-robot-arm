"""Exercise the real HTTP service with --simulate; never opens USB or GPIO."""
import json
from pathlib import Path
import re
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from urllib.request import Request, build_opener, ProxyHandler

ROOT = Path(__file__).resolve().parents[1]


class ManualHttpTests(unittest.TestCase):
    def test_simulated_arm_jog_release_stop(self):
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            port = listener.getsockname()[1]
        opener = build_opener(ProxyHandler({}))
        base = f'http://127.0.0.1:{port}'
        with tempfile.TemporaryDirectory() as directory:
            process = subprocess.Popen(
                [sys.executable, str(ROOT/'manual_control/server.py'), '--simulate', '--port', str(port),
                 '--log-dir', directory], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            try:
                deadline = time.monotonic()+8
                page = None
                while page is None and time.monotonic()<deadline:
                    try:
                        with opener.open(base+'/', timeout=.3) as response:
                            page = response.read().decode('utf-8')
                    except OSError:
                        time.sleep(.03)
                self.assertIsNotNone(page, 'Simulation service did not start')
                key = re.search(r"const\s+key\s*=\s*['\"]([^'\"]+)['\"]", page)[1]
                order = 0

                def state():
                    with opener.open(base+'/api/state', timeout=1) as response:
                        data = json.load(response)
                    self.assertTrue(data['simulated'], 'Refusing any action against a real service')
                    return data

                def action(name, **fields):
                    nonlocal order
                    order += 1
                    body = dict(key=key, client='offline-http-test', order=order, action=name, **fields)
                    request = Request(base+'/api/action', json.dumps(body).encode(),
                                      headers={'Content-Type':'application/json'})
                    with opener.open(request, timeout=2) as response:
                        self.assertTrue(json.load(response)['ok'])

                def wait(predicate, press=None, heartbeat=True):
                    end = time.monotonic()+5
                    while time.monotonic()<end:
                        current = state()
                        if predicate(current):
                            return current
                        if heartbeat and current['armed']:
                            action('heartbeat', press=press)
                        time.sleep(.04)
                    self.fail('Simulated state did not settle: '+str(current.get('message')))

                wait(lambda s: s['connected'] and len(s['axes'])==4, heartbeat=False)
                action('arm')
                ready = wait(lambda s: s['armed'] and s['axes'][0]['ready'])
                initial = ready['axes'][0]['position']
                action('jog', node=1, direction=1, speed=5, press='one-press')
                moved = wait(lambda s: abs(s['axes'][0]['position']-initial)>.1, press='one-press')
                self.assertTrue(moved['active'])
                action('release', press='one-press')
                stopped = wait(lambda s: not s['active'] and not s['stopping'] and s['axes'][0]['stationary'])
                self.assertNotEqual(stopped['axes'][0]['position'], initial)
                action('stop', disarm=True)
                wait(lambda s: not s['armed'] and not s['stopping'], heartbeat=False)
            finally:
                process.terminate()
                try:
                    _, error = process.communicate(timeout=4)
                except subprocess.TimeoutExpired:
                    process.kill()
                    _, error = process.communicate(timeout=4)
                if process.returncode not in (0, 1, -15, -9):
                    self.assertFalse(error, error.decode('utf-8', errors='replace'))


if __name__ == '__main__':
    unittest.main()
