import json
from pathlib import Path
import struct
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'manual_control'))
from telemetry import diagnostic
from server import Worker


def trace():
    return struct.pack('<IIqf6i3I',0x3147545a,123,-42,-12.5,-1000,180000,
                       800,780,1200,24000,1,0,1).ljust(64,b'\0')


def fault():
    return struct.pack('<I12i',0x31464349,1,124,-250,20,0,0,10,-20,30,40,-42,1).ljust(64,b'\0')


class TelemetryTests(unittest.TestCase):
    def test_signed_current_counts_and_units(self):
        n,kind,v=diagnostic(0x301,trace())
        self.assertEqual((n,kind,v['encoder_counts']),(1,'trace',-42))
        self.assertEqual(v['position_deg'],-1)
        self.assertEqual(v['iq_ref_ma'],800)
        self.assertEqual(v['speed_counts_s'],-12.5)
        self.assertEqual(diagnostic(0x381,fault())[2]['phase_a_counts'],-250)

    def test_bad_magic_and_length_rejected(self):
        for data in (b'',bytes(64),trace()[:56]):
            with self.assertRaises(ValueError):diagnostic(0x301,data)

    def test_worker_records_both_diagnostic_frames_without_transmitting(self):
        config=json.loads((Path(__file__).resolve().parents[1]/'config/manual-jog.json').read_text(encoding='utf-8'))
        frames=[SimpleNamespace(identifier=ident,data=data,fd=True,brs=True,extended=False,echo=False,error=False)
                for ident,data in [(0x301,trace()),(0x381,fault())]]
        class Fake:
            def recv(self,timeout):
                time.sleep(.001)
                return frames.pop(0) if frames else None
            def close(self):pass
        with tempfile.TemporaryDirectory() as folder:
            w=Worker(config,True,Path(folder))
            def connect():w.adapter=Fake();w.core.connected=True
            w.connect=connect
            w.thread.start()
            try:
                end=time.monotonic()+1
                while time.monotonic()<end:
                    with w.lock:complete='current_fault' in w.view.get('diagnostics',{}).get(1,{})
                    if complete:break
                    time.sleep(.01)
                with w.lock:v=w.view
                self.assertIn('trace',v['diagnostics'][1])
                self.assertIn('current_fault',v['diagnostics'][1])
                self.assertEqual(v['tx_count'],0)
            finally:
                w.shutdown.set();w.thread.join(2)
            records=[json.loads(line) for line in w.log_path.read_text(encoding='utf-8').splitlines()]
            self.assertEqual({r['id'] for r in records if r.get('event')=='rx'},{0x301,0x381})

    def test_receiver_drains_usb_while_control_thread_is_busy(self):
        config=json.loads((Path(__file__).resolve().parents[1]/'config/manual-jog.json').read_text(encoding='utf-8'))
        class Fake:
            def __init__(self):self.reads=0
            def recv(self,timeout):time.sleep(.001);self.reads+=1;return self.reads
            def close(self):pass
        with tempfile.TemporaryDirectory() as folder:
            w=Worker(config,False,Path(folder));w.adapter=Fake();w.start_receiver()
            try:
                time.sleep(.05)  # Control/logging work does not block USB ingestion.
                frame,received=w.read_frame()
                self.assertEqual(frame,1)
                self.assertLess(received,time.monotonic())
                self.assertGreater(w.adapter.reads,3)
            finally:w.close_adapter();w.log_file.close()


if __name__=='__main__':unittest.main()
