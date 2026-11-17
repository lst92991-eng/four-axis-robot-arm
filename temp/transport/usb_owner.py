"""用Linux文件锁保证同一USB-CAN只有一个进程控制。"""
import hashlib
import os
from pathlib import Path
import tempfile


class UsbOwner:
    def __init__(self, identity):
        self.file = None
        if os.name != 'posix':
            raise RuntimeError('USB-CAN实机独占入口仅支持Linux；Windows可运行离线模拟')
        import fcntl

        # 同一个物理适配器始终使用同一个锁文件
        key = hashlib.sha256(identity.encode('utf-8')).hexdigest()
        self.file = (Path(tempfile.gettempdir()) / ('stepfoc-usb-' + key + '.lock')).open('a')
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.close()
            raise RuntimeError('适配器正被另一进程使用，或无法取得独占锁')

    def close(self):
        # 关闭描述符释放锁，保留锁文件，避免出现同名的不同锁对象
        if self.file is not None:
            self.file.close()
            self.file = None
