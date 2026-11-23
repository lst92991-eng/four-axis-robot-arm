from types import SimpleNamespace

import os
import sys
from pathlib import Path

# 旧传输层按同目录导入 USB 独占模块。
sys.path.append(str(Path(__file__).resolve().parent))
from stepfoc_candlelight import Candlelight, Frame


class CandleBus:
    # 标定会话已有适配器时直接复用，否则按序列号打开
    def __init__(self, serial='', *, adapter=None):
        owner = None
        if os.name == 'nt':
            from windows_owner import UsbOwner as owner
        self.adapter = adapter if adapter is not None else Candlelight(serial=serial, fd=True, owner_factory=owner)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.adapter.close()

    def send(self, message, timeout=None):
        # OTA接口使用arbitration_id，底层统一使用标准CAN FD/BRS帧
        self.adapter.send(Frame(message.arbitration_id, message.data, fd=True, brs=True))

    def recv(self, timeout):
        # 丢弃本机发送回显，把收到的帧交回OTA接口
        frame = self.adapter.recv(timeout)
        if frame is not None and not frame.echo:
            return SimpleNamespace(arbitration_id=frame.identifier, data=frame.data,
                                   is_extended_id=frame.extended, is_error_frame=frame.error,
                                   is_fd=frame.fd)
        return None

