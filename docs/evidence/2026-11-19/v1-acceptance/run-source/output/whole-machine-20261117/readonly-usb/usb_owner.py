"""Process-wide Windows ownership for one physical adapter."""
import ctypes
import hashlib
import os
from ctypes import wintypes


class UsbOwner:
    def __init__(self, identity):
        self.handle = None
        if os.name != 'nt':
            return
        self.api = ctypes.WinDLL('kernel32', use_last_error=True)
        self.api.CreateMutexW.argtypes = [ctypes.c_void_p, wintypes.BOOL, wintypes.LPCWSTR]
        self.api.CreateMutexW.restype = wintypes.HANDLE
        self.api.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        self.api.WaitForSingleObject.restype = wintypes.DWORD
        self.api.ReleaseMutex.argtypes = [wintypes.HANDLE]
        self.api.CloseHandle.argtypes = [wintypes.HANDLE]
        key = hashlib.sha256(identity.encode('utf-8')).hexdigest()
        handle = self.api.CreateMutexW(None, False, 'Local\\StepFOC_USB_'+key)
        if not handle:
            raise ctypes.WinError(ctypes.get_last_error())
        status = self.api.WaitForSingleObject(handle, 0)
        if status not in (0, 0x80):
            self.api.CloseHandle(handle)
            raise RuntimeError('\u9002\u914d\u5668\u6b63\u88ab\u53e6\u4e00\u4e2a StepFOC \u7a97\u53e3\u4f7f\u7528\uff0c\u8bf7\u5148\u65ad\u5f00\u5176\u8fde\u63a5')
        self.handle = handle

    def close(self):
        if self.handle:
            self.api.ReleaseMutex(self.handle)
            self.api.CloseHandle(self.handle)
            self.handle = None
