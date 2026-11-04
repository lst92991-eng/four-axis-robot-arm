"""Four-node v3 CAN FD check. Transmits only PING (command 7)."""
import argparse
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import struct
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent / 'transport'))
from stepfoc_candlelight import (
    Candlelight, Frame, describe_bus_error, is_candlelight_status_report,
)

PACKET = struct.Struct('<4BI14i')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--seconds', type=int, default=60)
    parser.add_argument('--passive', type=int, default=8)
    args = parser.parse_args()
    root = Path(__file__).parent / 'comm-logs'
    root.mkdir(exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    raw_path = root / (stamp + '-frames.jsonl')
    summary_path = root / (stamp + '-summary.json')
    nodes = {n: dict(status=0, sent=0, accepted=0, rejected=0, timeout=0,
                     duplicates=0, resets=0, max_status_gap_ms=0,
                     power_seen=False, configured_seen=False, builds=[],
                     faults=[], states=[], encoder_valid_values=[],
                     status_without_brs=0, rtt_ms=[])
             for n in range(1, 5)}
    pending = {}
    completed = set()
    last_host, last_board = {}, {}
    counts = Counter()
    errors, invalid = [], []
    started = time.monotonic()
    adapter = None
    exception = None
    passive_ok = False
    try:
        adapter = Candlelight(serial=args.serial, fd=True)
        print('ADAPTER ' + adapter.label, flush=True)
        with raw_path.open('x', encoding='utf-8') as raw:
            def record(direction, frame):
                raw.write(json.dumps(dict(t=round(time.monotonic()-started, 6),
                    direction=direction, id=frame.identifier, fd=frame.fd,
                    brs=frame.brs, extended=frame.extended, error=frame.error,
                    echo=frame.echo, data=bytes(frame.data).hex())) + '\n')

            def receive(timeout):
                frame = adapter.recv(timeout)
                if frame is None:
                    return
                record('rx', frame)
                if frame.echo:
                    counts['echo'] += 1
                    return
                if frame.error:
                    if is_candlelight_status_report(frame):
                        counts['adapter_status_report'] += 1
                    else:
                        errors.append(describe_bus_error(frame))
                    return
                counts[f'0x{frame.identifier:03x}'] += 1
                n = frame.identifier - 0x200
                reply_node = frame.identifier - 0x480
                if n not in nodes and reply_node not in nodes:
                    return
                if frame.extended or not frame.fd or len(frame.data) != 64:
                    invalid.append('Unexpected frame format')
                    return
                kind, node, result, version, seq, *values = PACKET.unpack(frame.data)
                if version != 3 or node != (n if n in nodes else reply_node):
                    invalid.append('Node/version mismatch')
                    return
                now = time.monotonic()
                if n in nodes:
                    if kind != 0x40 or result != 0:
                        invalid.append('Invalid STATUS header')
                        return
                    s = nodes[n]
                    s['status'] += 1
                    s['status_without_brs'] += not frame.brs
                    if n in last_host:
                        s['max_status_gap_ms'] = max(s['max_status_gap_ms'],
                                                    (now-last_host[n])*1000)
                    board_ms = values[0] & 0xffffffff
                    if n in last_board and (board_ms-last_board[n]) & 0xffffffff > 0x80000000:
                        s['resets'] += 1
                    last_host[n], last_board[n] = now, board_ms
                    status = values[9] & 0xffffffff
                    flags = status >> 16
                    s['power_seen'] |= bool(flags & 64)
                    s['configured_seen'] |= bool(flags & 16)
                    for key, value in [('builds', f'{values[10] & 0xffffffff:08x}'),
                                       ('faults', (status >> 8) & 255),
                                       ('states', status & 255),
                                       ('encoder_valid_values', bool(flags & 32))]:
                        if value not in s[key]:
                            s[key].append(value)
                    s['last_board_ms'] = board_ms
                    return
                if kind != 0x87:
                    invalid.append('Unexpected command reply')
                    return
                key = (node, seq)
                if key in completed:
                    nodes[node]['duplicates'] += 1
                elif key in pending:
                    nodes[node]['rtt_ms'].append((now-pending.pop(key))*1000)
                    nodes[node]['accepted' if result == 0 else 'rejected'] += 1
                    completed.add(key)
                else:
                    counts['unmatched_ping_reply'] += 1

            until = time.monotonic() + args.passive
            while time.monotonic() < until:
                receive(.05)
            print('PASSIVE ' + json.dumps(nodes), flush=True)
            passive_ok = all(s['status'] > 0 and not s['power_seen'] and
                             not s['configured_seen'] for s in nodes.values())
            if passive_ok and args.seconds > 0:
                # Persistent monotonic sequence within the signed half-range.
                seq_path = root / 'ping-sequence.json'
                sequence = json.loads(seq_path.read_text()) if seq_path.exists() else 0
                until = time.monotonic() + args.seconds
                next_send = time.monotonic()
                next_report = next_send + 15
                while time.monotonic() < until:
                    now = time.monotonic()
                    if now >= next_send:
                        sequence += 1
                        if sequence >= 0x7fffffff:
                            raise RuntimeError('PING sequence exhausted')
                        seq_path.write_text(json.dumps(sequence))
                        for node in nodes:
                            data = PACKET.pack(7, node, 0, 3, sequence, *([0]*14))
                            frame = Frame(0x400+node, data, fd=True, brs=True)
                            pending[(node, sequence)] = time.monotonic()
                            adapter.send(frame)
                            record('tx', frame)
                            nodes[node]['sent'] += 1
                        next_send = time.monotonic() + .2
                    receive(.01)
                    now = time.monotonic()
                    for key, sent_at in list(pending.items()):
                        if now-sent_at > 1:
                            nodes[key[0]]['timeout'] += 1
                            del pending[key]
                    if now >= next_report:
                        print('PROGRESS ' + json.dumps({n: {k:s[k] for k in
                            ('status','sent','accepted','rejected','timeout')}
                            for n,s in nodes.items()}), flush=True)
                        next_report = now + 15
                until = time.monotonic() + 1.1
                while time.monotonic() < until:
                    receive(.01)
                for node, _ in pending:
                    nodes[node]['timeout'] += 1
    except Exception as exc:
        exception = repr(exc)
        print('ERROR ' + exception, flush=True)
    finally:
        io_stats = dict(adapter.io_stats) if adapter else {}
        if adapter:
            try:
                adapter.close()
            except Exception as exc:
                exception = (exception or '') + '; close: ' + repr(exc)
        for s in nodes.values():
            rtt = s.pop('rtt_ms')
            s['rtt_mean_ms'] = sum(rtt)/len(rtt) if rtt else None
            s['rtt_max_ms'] = max(rtt) if rtt else None
        passed = (passive_ok and args.seconds > 0 and not exception and not errors
                  and not invalid and all(s['sent'] > 0 and s['accepted'] == s['sent']
                  and not s['timeout'] and not s['rejected'] and not s['duplicates']
                  and not s['resets'] and not s['power_seen']
                  and not s['configured_seen'] and len(s['builds']) == 1
                  and not s['status_without_brs'] and s['max_status_gap_ms'] < 250
                  for s in nodes.values()))
        report = dict(passed=passed, elapsed_s=time.monotonic()-started,
                      bitrate=500000, data_bitrate=2000000,
                      commands_sent=['PING'] if any(s['sent'] for s in nodes.values()) else [],
                      nodes=nodes, counts=dict(counts), errors=errors,
                      invalid=invalid, exception=exception, io_stats=io_stats,
                      raw_path=str(raw_path.resolve()))
        summary_path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print('SUMMARY ' + str(summary_path.resolve()), flush=True)
        print(json.dumps(report), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
