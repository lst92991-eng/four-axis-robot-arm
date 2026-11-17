import importlib.util
import hashlib
import os
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest
import uuid
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'temp/transport'))
from stepfoc_candlelight import Candlelight,Frame,pack_frame
from usb_owner import UsbOwner
from candle_bus import CandleBus


@unittest.skipUnless(importlib.util.find_spec('usb'),'requires CAN environment pyusb')
class StreamTests(unittest.TestCase):
    def test_split_and_coalesced_frames_in_bulk_reads(self):
        import usb.core
        expected=[Frame(0x201,bytes(range(64)),fd=True,brs=True),
                  Frame(0x123,b'abcdefgh'),Frame(0x301,b'X'*64,fd=True,brs=True)]
        raw=b''.join(pack_frame(f,0xffffffff) for f in expected)
        chunks=[raw[:64],raw[64:92],raw[92:]]
        class Fake:
            def read(self,endpoint,size,timeout):
                self.last_size=size
                if chunks:return chunks.pop(0)
                raise usb.core.USBTimeoutError('no data')
        c=Candlelight.__new__(Candlelight);c.dev=Fake();c.ep_in=0x81;c.read_size=512
        c.rx_buffer=bytearray();c.partial_since=None
        for want in expected:
            got=c.recv(.1)
            self.assertEqual((got.identifier,got.data,got.fd,got.brs,got.echo),
                             (want.identifier,want.data,want.fd,want.brs,False))
        self.assertEqual(c.dev.last_size,512)
        self.assertIsNone(c.recv(.001))


@unittest.skipUnless(os.name == 'posix', 'Linux adapter ownership uses fcntl')
class UsbOwnerTests(unittest.TestCase):
    def test_other_process_is_blocked_until_owner_closes(self):
        identity = 'offline-lock-test-' + uuid.uuid4().hex
        owner = UsbOwner(identity)
        transport = Path(__file__).resolve().parents[1] / 'temp/transport'
        child = ('import sys\nfrom usb_owner import UsbOwner\n'
                 'try: owner=UsbOwner(sys.argv[1])\n'
                 'except RuntimeError: sys.exit(23)\n'
                 'owner.close()\n')
        try:
            held = subprocess.run([sys.executable, '-c', child, identity], cwd=transport,
                                  capture_output=True, timeout=3)
            self.assertEqual(held.returncode, 23, held.stderr)
            owner.close()
            released = subprocess.run([sys.executable, '-c', child, identity], cwd=transport,
                                      capture_output=True, timeout=3)
            self.assertEqual(released.returncode, 0, released.stderr)
            owner.close()
        finally:
            owner.close()
            key = hashlib.sha256(identity.encode()).hexdigest()
            (Path(tempfile.gettempdir()) / ('stepfoc-usb-' + key + '.lock')).unlink(missing_ok=True)


class BusAdapterTests(unittest.TestCase):
    def test_existing_adapter_is_reused_for_send_receive_and_close(self):
        adapter = Mock()
        adapter.recv.return_value = Frame(0x201, b'1234', fd=True, brs=True)
        with CandleBus(adapter=adapter) as bus:
            bus.send(SimpleNamespace(arbitration_id=0x401, data=b'command'))
            frame = adapter.send.call_args.args[0]
            self.assertEqual((frame.identifier, frame.data, frame.fd, frame.brs), (0x401, b'command', True, True))
            response = bus.recv(.1)
            self.assertEqual((response.arbitration_id, response.data, response.is_fd), (0x201, b'1234', True))
            adapter.recv.return_value.echo = True
            self.assertIsNone(bus.recv(.1))
        adapter.close.assert_called_once()


if __name__=='__main__':unittest.main()
