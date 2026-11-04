from types import SimpleNamespace

import sys
from pathlib import Path

# 旧传输层按同目录导入 USB 独占模块。
sys.path.append(str(Path(__file__).resolve().parent))
from stepfoc_candlelight import Candlelight, Frame


class CandleBus:
    def __init__(self, serial):
        self.adapter = Candlelight(serial=serial, fd=True)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.adapter.close()

    def send(self, message, timeout=None):
        self.adapter.send(Frame(message.arbitration_id, message.data, fd=True, brs=True))

    def recv(self, timeout):
        frame = self.adapter.recv(timeout)
        if frame is not None and not frame.echo:
            return SimpleNamespace(arbitration_id=frame.identifier, data=frame.data,
                                   is_extended_id=frame.extended, is_error_frame=frame.error,
                                   is_fd=frame.fd)
        return None

