"""USB-CAN v3 executor for the live Windows bench. All angles are driver degrees."""

from datetime import datetime
from pathlib import Path
import json
import sys
import time

WORK = Path(__file__).resolve().parents[1]
ROOT = WORK.parents[1]
sys.path.insert(0, str(WORK / 'readonly-usb'))
sys.path.insert(1, str(ROOT / 'manual_control'))
from stepfoc_candlelight import Candlelight, Frame, is_candlelight_status_report
from core import CLEAR, MOVE, OFF, PACKET, PING, STOP, ZERO, packet

PROFILE = json.loads(Path(__file__).with_name('j1-fast-profile.json').read_text(encoding='utf8'))
Z_ORIGIN = PROFILE['saved_travel_deg']
SET_SPEED_GAINS = 10
SET_PROFILE = 13
NODES = (1, 2, 3)
BUILDS = {1: PROFILE['build_id'], 2: '41f870bf', 3: '41f870bf'}
SERIAL = '37813482333551062'


class Drive:
    def __init__(self):
        self.can = Candlelight(serial=SERIAL, fd=True)
        self.states = {}
        self.sequences = {n: int(time.time() * 1000) % 0x3fffffff + 20 for n in NODES}
        self.acks = {}
        self.last_sent = {n: 0 for n in NODES}
        self.last_update = {}
        self.on_poll = None
        self.profile_set = False
        self.sync_ping = None
        self.trace = (WORK / 'usb-game-drive.jsonl').open('a', encoding='utf-8')

    def note(self, event, **fields):
        self.trace.write(json.dumps(dict(time=datetime.now().astimezone().isoformat(), event=event, **fields),
                                    ensure_ascii=False) + '\n')
        self.trace.flush()

    def send(self, n, kind, a=0, b=0):
        seq = (self.sequences[n] + 1) & 0xFFFFFFFF
        self.sequences[n] = seq
        wire_angle = a - round(Z_ORIGIN * 1000) if n == 1 and kind in (MOVE, ZERO) else a
        self.can.send(Frame(0x400 + n, packet(kind, n, seq, wire_angle, b), fd=True, brs=True))
        self.last_sent[n] = time.monotonic()
        if kind != PING:
            self.note('tx', node=n, kind=kind, sequence=seq, a=a, b=b)
        return seq

    def poll(self, active=True):
        if self.on_poll is not None:
            self.on_poll()
        now = time.monotonic()
        if active:
            for n in NODES:
                a = self.states.get(n, {})
                if a.get('state') in (1, 2, 3, 5) and now - self.last_sent[n] > .10:
                    self.send(n, PING)
        f = self.can.recv(.028)
        if not f or f.echo:
            return
        if f.error:
            if not is_candlelight_status_report(f):
                self.note('can_controller_report', identifier=f.identifier, data=f.data.hex())
            if f.identifier & 0x40:
                raise RuntimeError('USB-CAN bus-off')
            return
        if f.extended or not f.fd or not f.brs or len(f.data) != PACKET.size:
            return
        k, n, result, version, seq, *v = PACKET.unpack(f.data)
        if n not in NODES or version != 3:
            return
        if f.identifier == 0x480 + n:
            if k != (PING | 0x80) or (n, seq) == self.sync_ping:
                self.acks[n, k, seq] = result
            return
        if f.identifier != 0x200 + n or k != 0x40 or result != 0:
            return
        flags = (v[9] & 0xffffffff) >> 16
        origin = Z_ORIGIN if n == 1 else 0
        item = dict(position=origin + v[1] / 1000, target=origin + v[2] / 1000, velocity=v[4] / 1000,
                    counts=v[5], uptime=v[0] & 0xffffffff, build=f'{v[10] & 0xffffffff:08x}',
                    state=v[9] & 255, fault=v[9] >> 8 & 255, detail=v[3] >> 24 & 255,
                    bus_mv=v[12], referenced=bool(flags & 2),
                    interlocked=bool(flags & 8), configured=bool(flags & 16), encoder_valid=bool(flags & 32),
                    power=bool(flags & 64), stationary=bool(flags & 256))
        self.states[n] = item
        self.last_update[n] = time.monotonic()

    def healthy(self):
        now = time.monotonic()
        for n in NODES:
            a = self.states.get(n)
            if a is None or now - self.last_update[n] > .25:
                raise RuntimeError(f'J{n} status expired')
            bus_max = PROFILE.get('running_bus_max_mv', 30000) if n == 1 and a['power'] else 30000
            if a['build'] != BUILDS[n] or a['fault'] or a['interlocked'] or not a['configured'] \
                    or not a['referenced'] or not a['encoder_valid'] or not 10000 <= a['bus_mv'] <= bus_max:
                raise RuntimeError(f'J{n} state unavailable: {a}')

    def wait(self, condition, seconds, check=True, elapsed_ok=False):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self.poll()
            if check:
                self.healthy()
            if condition():
                return
        if not elapsed_ok:
            raise TimeoutError('CAN state or ACK timeout')

    def read_initial(self):
        self.wait(lambda: len(self.states) == 3 and
                  all(time.monotonic() - self.last_update[n] < .18 for n in NODES), .75, check=False)
        # CAN watchdog keeps the two rotary targets energized when the host is idle.
        for n in (2, 3):
            axis = self.states[n]
            if (axis['interlocked'] and axis['state'] == 1 and axis['power'] and axis['stationary']
                    and axis['build'] == BUILDS[n] and axis['configured'] and axis['referenced']
                    and axis['encoder_valid'] and not axis['fault']):
                self.command(n, CLEAR, check=False)
                self.wait(lambda: not self.states[n]['interlocked'], .6, check=False)
        self.healthy()
        self.wait(lambda: all(a['state'] in (0, 1) and a['stationary'] for a in self.states.values()), 2)
        if not self.profile_set:
            for n in NODES:
                self.sync_sequence(n)
            self.command(1, SET_PROFILE, round(PROFILE['brake_deg_s2'] * 1000),
                         round(PROFILE['acceleration_deg_s2'] * 1000))
            # Apply the tested J3 gains on a fresh boot while the board is already OFF.
            if self.states[3]['state'] == 0 and not self.states[3]['power']:
                self.command(3, SET_SPEED_GAINS, 1000, 35000)
            self.profile_set = True
        self.note('initial', axes=self.states)
        return self.states

    def sync_sequence(self, n):
        try:
            for _ in range(4):
                self.sync_ping = n, (self.sequences[n] + 1) & 0xFFFFFFFF
                seq = self.send(n, PING)
                if self.ack(n, PING, seq, allow_reject=True):
                    return
                self.sequences[n] = (seq + 0x3FFFFFFF) & 0xFFFFFFFF
            raise RuntimeError(f'J{n} sequence sync rejected')
        finally:
            self.sync_ping = None

    def command(self, n, kind, a=0, b=0, check=True):
        seq = self.send(n, kind, a, b)
        self.ack(n, kind, seq, check)

    def ack(self, n, kind, seq, check=True, allow_reject=False):
        key = n, kind | 0x80, seq
        self.wait(lambda: key in self.acks, .45, check=check)
        if self.acks.pop(key) != 0:
            self.note('rejected', node=n, kind=kind, sequence=seq, actual=self.states[n])
            if allow_reject:
                return False
            raise RuntimeError(f'J{n} rejected CAN action {kind}: {self.states[n]}')
        self.note('ack', node=n, kind=kind, sequence=seq)
        return True

    def move(self, n, goal, speed, tolerance=.12, *, on_rise=None):
        self.wait(lambda: self.states[n]['state'] in (0, 1) and self.states[n]['stationary'], 2)
        initial = self.states[n]['position']
        distance = abs(goal - initial)
        if distance <= tolerance:
            return
        if self.states[n]['power']:
            self.idle(n)
        print(f'J{n} MOVE {initial:.3f} -> {goal:.3f} degrees', flush=True)
        seen = self.states[n]['uptime']
        for attempt in range(4):
            seq = self.send(n, MOVE, round(goal * 1000), round(speed * 1000))
            if self.ack(n, MOVE, seq, allow_reject=n == 1):
                break
            stamp = self.states[n]['uptime']
            self.wait(lambda: self._off_ready(n, stamp), .75)
        else:
            raise RuntimeError('J1 position request repeatedly rejected')

        def arrived():
            nonlocal seen
            a = self.states[n]
            if on_rise and a['state'] == 2 and a['power'] and a['velocity'] > .05:
                if 0 < ((a['uptime'] - seen) & 0xFFFFFFFF) < 0x80000000:
                    seen = a['uptime']
                    on_rise(a.copy())
                    self.healthy()
                    a = self.states[n]
            target_matches = abs(a['target'] - goal) <= .01 and abs(a['position'] - goal) <= tolerance
            settled = a['stationary'] and a['state'] == 1
            slow_near_goal = n == 3 and a['state'] == 2 and abs(a['velocity']) <= .20
            return target_matches and (settled or slow_near_goal)
        ramps = (speed / PROFILE['acceleration_deg_s2'] + speed / PROFILE['brake_deg_s2']
                 if n == 1 else 2 * speed / 30)
        self.wait(arrived, distance / speed + ramps + 7)
        if self.states[n]['state'] == 2:
            self.command(n, STOP)
            self.wait(lambda: self.states[n]['stationary'] and self.states[n]['state'] == 1, 3)
            if abs(self.states[n]['position'] - goal) > .20:
                raise RuntimeError(f'J{n} stopped away from the requested position')
        self.note('arrived', node=n, goal=goal, actual=self.states[n])

    def idle(self, n):
        self.command(n, OFF)
        acknowledged_at = self.states[n]['uptime']
        self.wait(lambda: self._off_ready(n, acknowledged_at), 2)

    def _off_ready(self, n, after_uptime):
        a = self.states[n]
        fresh = 0 < ((a['uptime'] - after_uptime) & 0xFFFFFFFF) < 0x80000000
        return fresh and a['state'] == 0 and not a['power'] and a['stationary'] and abs(a['velocity']) <= .05

    def xy(self, shoulder_goal, elbow_goal, shoulder_speed, elbow_speed=30.0):
        """Give both rotary drives an absolute target, then leave them in HOLD."""
        nodes = (2, 3)
        goals = {2: round(shoulder_goal * 1000), 3: round(elbow_goal * 1000)}
        speeds = {2: shoulder_speed, 3: elbow_speed}
        self.wait(lambda: all(self.states[n]['state'] in (0, 1) and self.states[n]['stationary']
                              for n in nodes), 2)
        initial = {n: self.states[n].copy() for n in nodes}
        started = time.monotonic()
        sequences = {n: self.send(n, MOVE, goals[n], round(speeds[n] * 1000)) for n in nodes}
        self.note('xy_position_batch', goals=goals, speeds=speeds,
                  transmit_ms=(time.monotonic() - started) * 1000, initial=initial)
        for n in nodes:
            self.ack(n, MOVE, sequences[n])

        def held_at_target(n):
            a = self.states[n]
            return (a['uptime'] != initial[n]['uptime'] and a['state'] == 1 and a['power']
                    and a['stationary'] and abs(a['target'] - goals[n] / 1000) <= .001
                    and abs(a['position'] - goals[n] / 1000) <= .101)

        timeout = max(abs(goals[n] / 1000 - initial[n]['position']) / speeds[n]
                      + 2 * speeds[n] / 30 for n in nodes) + 7
        self.wait(lambda: all(held_at_target(n) for n in nodes), timeout)
        self.note('xy_position_held', goals=goals, actual={n:self.states[n] for n in nodes},
                  elapsed_s=time.monotonic() - started)

    def z(self, goal, high=-19353.34, precision=4.5, speed_limit=90.0, *, on_rise=None):
        if not self.profile_set:
            raise RuntimeError('J1 fast profile was not applied')
        self.move(1, goal, speed_limit, precision, on_rise=on_rise)

    def stop(self):
        failed = False
        before = {n: a['uptime'] for n, a in self.states.items()}
        # Issue STOP independently per moving axis, even if another axis has a fault.
        for n in NODES:
            try:
                self.send(n, STOP)
            except Exception as e:
                self.note('stop_send_failed', node=n, error=repr(e))
                failed = True
        until = time.monotonic() + 3
        while time.monotonic() < until:
            try:
                self.poll()
            except Exception:
                failed = True
            if len(self.states) == 3 and all(time.monotonic() - self.last_update[n] < .25 and
                    self.states[n]['uptime'] != before.get(n) and
                    a['stationary'] and (a['state'] in (0, 4) and not a['power'] or a['state'] == 1)
                    for n, a in self.states.items()):
                self.note('stopped', axes=self.states)
                return not failed
        self.note('stop_not_confirmed', axes=self.states)
        return False

    def close(self):
        self.can.close()
        self.trace.close()

    def finish_park(self, save=False):
        # Park STOP already ran. Finish the normal disarm using fresh feedback.
        def stopped_without_fault():
            return len(self.states) == 3 and all(
                a['state'] in (0, 1) and a['stationary'] and not a['fault'] and a['referenced']
                and a['encoder_valid'] and a['build'] == BUILDS[n] for n, a in self.states.items()
            )
        self.wait(stopped_without_fault, 3, check=False)
        for n in NODES:
            a = self.states[n]
            if a['interlocked'] and n != 1:
                if a['state'] != 1:
                    raise RuntimeError(f'J{n} cannot resume the hold watchdog handshake')
                self.command(n, CLEAR, check=False)
                self.wait(lambda: not self.states[n]['interlocked'], .6, check=False)
            if self.states[n]['power']:
                self.command(n, OFF, check=False)
                self.wait(lambda: self.states[n]['state'] == 0 and not self.states[n]['power'], .6, check=False)
            # The older J1 image requires OFF and a same-angle ZERO to clear a hold lock.
            # Retain the multi-turn reference and verify the raw count has not moved.
            if n == 1 and self.states[n]['interlocked']:
                before = self.states[n].copy()
                self.wait(lambda: self.states[n]['uptime'] != before['uptime'], .5, check=False)
                after = self.states[n]
                if not after['stationary'] or abs(after['counts'] - before['counts']) > 2 or after['state'] != 0:
                    raise RuntimeError('J1 position changed while releasing stationary lock')
                self.command(1, ZERO, round(after['position'] * 1000), check=False)
                self.wait(lambda: self.states[1]['referenced'] and not self.states[1]['interlocked'] and
                          abs(self.states[1]['position'] - after['position']) < .10, .6, check=False)
        self.read_initial()
        if save:
            (WORK / 'usb-game-park-readback.json').write_text(
                json.dumps(self.states, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
            print(json.dumps(self.states, ensure_ascii=False), flush=True)
