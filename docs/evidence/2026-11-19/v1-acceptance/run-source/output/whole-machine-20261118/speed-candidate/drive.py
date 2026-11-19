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
from core import CLEAR, MOVE, OFF, PACKET, PING, STOP, VELOCITY, ZERO, packet

PROFILE = json.loads(Path(__file__).with_name('j1-fast-profile.json').read_text(encoding='utf8'))
Z_ORIGIN = PROFILE['saved_travel_deg']
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

    def send(self, n, kind, a=0, b=0, *, c=0):
        seq = (self.sequences[n] + 1) & 0xFFFFFFFF
        self.sequences[n] = seq
        wire_angle = a - round(Z_ORIGIN * 1000) if n == 1 and kind in (MOVE, ZERO) else a
        data = packet(kind, n, seq, wire_angle, b)
        if c:
            data = data[:16] + c.to_bytes(4, 'little', signed=True) + data[20:]
        self.can.send(Frame(0x400 + n, data, fd=True, brs=True))
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
        if n == 1 and item['fault'] == 5 and item['detail'] == 4 and v[11]:
            item['following_trip_mdeg'] = ((v[11] & 65535) ^ 32768) - 32768
            item['following_near_ms'] = (v[11] & 0xFFFFFFFF) >> 16
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
        self.healthy()
        self.wait(lambda: all(a['state'] in (0, 1) and a['stationary'] for a in self.states.values()), 2)
        if not self.profile_set:
            for n in NODES:
                self.sync_sequence(n)
            self.command(1, SET_PROFILE, round(PROFILE['brake_deg_s2'] * 1000),
                         round(PROFILE['acceleration_deg_s2'] * 1000))
            self.profile_set = True
        self.note('initial', axes=self.states)
        return self.states

    def sync_sequence(self, n, check=True):
        try:
            for _ in range(4):
                self.sync_ping = n, (self.sequences[n] + 1) & 0xFFFFFFFF
                seq = self.send(n, PING)
                if self.ack(n, PING, seq, check=check, allow_reject=True):
                    return
                self.sequences[n] = (seq + 0x3FFFFFFF) & 0xFFFFFFFF
            raise RuntimeError(f'J{n} sequence sync rejected')
        finally:
            self.sync_ping = None

    def command(self, n, kind, a=0, b=0, check=True, *, c=0):
        seq = self.send(n, kind, a, b, c=c)
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
            if self.ack(n, MOVE, seq, allow_reject=True):
                break
            stamp = self.states[n]['uptime']
            self.wait(lambda: self._off_ready(n, stamp), .75)
        else:
            raise RuntimeError(f'J{n} position request repeatedly rejected')
        stable_since = None

        def arrived():
            nonlocal seen, stable_since
            a = self.states[n]
            if on_rise and a['state'] == 2 and a['power'] and a['velocity'] > .05:
                if 0 < ((a['uptime'] - seen) & 0xFFFFFFFF) < 0x80000000:
                    seen = a['uptime']
                    on_rise(a.copy())
                    self.healthy()
                    a = self.states[n]
            target_matches = abs(a['target'] - goal) <= .01 and abs(a['position'] - goal) <= tolerance
            settled = a['stationary'] and a['state'] == 1
            if n == 3 and a['state'] == 2 and target_matches and a['stationary']:
                if stable_since is None:
                    stable_since = a['uptime']
                return ((a['uptime'] - stable_since) & 0xFFFFFFFF) >= 150
            stable_since = None
            return target_matches and settled
        ramps = (speed / PROFILE['acceleration_deg_s2'] + speed / PROFILE['brake_deg_s2']
                 if n == 1 else 2 * speed / 30)
        self.wait(arrived, distance / speed + ramps + 7)
        if self.states[n]['state'] == 2:
            self.idle(n)
            if abs(self.states[n]['position'] - goal) > tolerance:
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

    def xy(self, shoulder_goal, elbow_goal, shoulder_speed, park=False):
        shoulder_tolerance = .5 if park else .12
        elbow_tolerance = 4.0 if park else 1.2
        self.wait(lambda: all(self.states[n]['stationary'] for n in (2, 3)), 2)
        move_shoulder = abs(self.states[2]['position'] - shoulder_goal) > shoulder_tolerance
        move_elbow = abs(self.states[3]['position'] - elbow_goal) > elbow_tolerance
        if not move_shoulder and not move_elbow:
            return

        released = {}
        for n in (2, 3):
            if self.states[n]['state'] != 0 or self.states[n]['power']:
                released[n] = self.send(n, OFF)
        for n, seq in released.items():
            self.ack(n, OFF, seq)
        # OFF can be acknowledged one control cycle before the output stage actually drops power.
        acknowledged_at = {n: self.states[n]['uptime'] for n in (2, 3)}
        self.wait(lambda: all(self._off_ready(n, acknowledged_at[n]) for n in (2, 3)), 2)

        move_shoulder = abs(self.states[2]['position'] - shoulder_goal) > shoulder_tolerance
        initial = self.states[3]['position']
        delta = elbow_goal - initial
        move_elbow = abs(delta) > elbow_tolerance
        direction = 1 if delta > 0 else -1
        large = abs(delta) > 23
        elbow_speed = 30 if large else min(9, max(6, abs(delta) * 1.5))
        started = time.monotonic()
        elbow_seq = self.send(3, VELOCITY, round(direction * elbow_speed * 1000)) if move_elbow else None
        shoulder_seq = self.send(2, MOVE, round(shoulder_goal * 1000), round(shoulder_speed * 1000)) if move_shoulder else None
        self.note('xy_batch', shoulder=shoulder_goal, elbow=elbow_goal, sequences=(shoulder_seq, elbow_seq),
                  transmit_ms=(time.monotonic() - started) * 1000)
        if shoulder_seq is not None:
            self.ack(2, MOVE, shoulder_seq)
        if elbow_seq is not None:
            accepted = self.ack(3, VELOCITY, elbow_seq, allow_reject=True)
            for attempt in range(5):
                if accepted:
                    break
                rejected_at = self.states[3]['uptime']
                self.wait(lambda: self._off_ready(3, rejected_at), .75)
                initial = self.states[3]['position']
                delta = elbow_goal - initial
                move_elbow = abs(delta) > elbow_tolerance
                if not move_elbow:
                    break
                direction = 1 if delta > 0 else -1
                large = abs(delta) > 23
                elbow_speed = 30 if large else min(9, max(6, abs(delta) * 1.5))
                elbow_seq = self.send(3, VELOCITY, round(direction * elbow_speed * 1000))
                accepted = self.ack(3, VELOCITY, elbow_seq, allow_reject=True)
            if move_elbow and not accepted:
                raise RuntimeError('J3 repeated velocity rejection')

        braking = None
        def at_goals():
            nonlocal braking
            a, b = self.states[2], self.states[3]
            if move_elbow and braking is None:
                if direction * (b['position'] - initial) < -5:
                    raise RuntimeError('J3 moved away from the target')
                speed = max(0, direction * b['velocity'])
                lead = max(5, .62 * speed) if large else max(.35, speed * speed / 60 + .04 * speed)
                if direction * (elbow_goal - b['position']) <= lead:
                    self.note('elbow_brake_request', goal=elbow_goal, actual=b)
                    braking = self.send(3, STOP)
            shoulder_ready = not move_shoulder or (
                a['state'] == 1 and a['stationary'] and abs(a['position'] - shoulder_goal) <= shoulder_tolerance)
            elbow_ready = not move_elbow or (braking is not None and b['state'] == 1 and b['stationary'])
            return shoulder_ready and elbow_ready

        shoulder_distance = abs(shoulder_goal - self.states[2]['position'])
        self.wait(at_goals, max(shoulder_distance / shoulder_speed, abs(delta) / 8) + 6)
        if braking is not None:
            self.ack(3, STOP, braking)
        self.elbow(elbow_goal, tolerance=elbow_tolerance)
        if abs(self.states[2]['position'] - shoulder_goal) > max(shoulder_tolerance, .18):
            raise RuntimeError('J2 shifted after the coordinated travel')
        self.note('xy_arrived', shoulder=shoulder_goal, elbow=elbow_goal, actual=[self.states[n] for n in (2, 3)],
                  elapsed_s=time.monotonic() - started)

    def xy_position(self, shoulder_goal, elbow_goal, shoulder_speed, elbow_speed=30.0):
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

    def elbow(self, goal, tolerance=1.2):
        if abs(goal - self.states[3]['position']) > tolerance:
            self.move(3, goal, 8, tolerance)

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

    def recover_j3_following(self):
        self.wait(lambda: len(self.states) == 3 and all(a['stationary'] for a in self.states.values()),
                  3, check=False)
        before = self.states[3].copy()
        if before['fault']:
            if (before['state'], before['fault'], before['power'], before['build']) != (4, 5, False, BUILDS[3]) \
                    or not before['encoder_valid'] or not before['referenced']:
                raise RuntimeError(f'J3 reported a different fault: {before}')
            self.command(3, CLEAR, check=False)
            self.wait(lambda: self.states[3]['state'] == 0 and not self.states[3]['fault'], 1, check=False)
            after = self.states[3]
            if not after['referenced'] or abs(after['counts'] - before['counts']) > 2 \
                    or abs(after['position'] - before['position']) > .07:
                raise RuntimeError(f'J3 reference changed after CLEAR: {before} -> {after}')
            self.note('j3_following_recovered', before=before, after=after)
        self.finish_park()
        print('J3 following fault cleared, current encoder reference verified.', flush=True)


def park():
    pose = json.loads((WORK / 'live-game-observation-pose.json').read_text(encoding='utf-8'))
    driver = Drive()
    try:
        current = driver.read_initial()
        if not -20286.5 <= current[1]['position'] <= -19341.3:
            raise RuntimeError('J1 is outside the taught pickup-to-travel range')
        driver.z(-19353.34, speed_limit=90)
        if driver.states[1]['position'] < -19359:
            raise RuntimeError('J1 has not cleared the pieces')
        driver.move(2, pose['j2_feedback_deg'], 8)
        driver.move(3, pose['j3_motor_deg'], 10)
        for n in (2, 3, 1):
            if driver.states[n]['power']:
                driver.idle(n)
        driver.read_initial()
        (WORK / 'usb-game-park-readback.json').write_text(
            json.dumps(driver.states, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print('Observation pose reached and driver feedback checked.', flush=True)
    except BaseException:
        try:
            if not driver.stop():
                print('STOP not fully confirmed.', flush=True)
        finally:
            raise
    finally:
        driver.close()


if __name__ == '__main__':
    if sys.argv[1:] == ['park']:
        park()
    elif sys.argv[1:] in (['finish-park'], ['recover-j3']):
        driver = Drive()
        try:
            if sys.argv[1] == 'recover-j3':
                driver.recover_j3_following()
            else:
                driver.finish_park(save=True)
        finally:
            driver.close()
    else:
        raise SystemExit('Usage: drive.py park | finish-park | recover-j3')
