import importlib.util
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'temp/transport'))
from stepfoc_candlelight import Candlelight,Frame,pack_frame


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


if __name__=='__main__':unittest.main()
