"""CAN错误解释及人工可读故障日志；不发送任何设备命令。"""
import json
import threading
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path

LOG_PATH = Path(__file__).resolve().parents[2] / 'output/hardware-validation/z-can-diagnostics.log'
_lock = threading.Lock()


def append_diagnostic(level, message, **details):
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='milliseconds')
    data = {'monotonic_ns': time.monotonic_ns(), **details}
    line = '%s [%s] %s | %s\n' % (stamp, level, str(message).replace('\n', ' / '), json.dumps(data, ensure_ascii=False))
    with _lock, LOG_PATH.open('a', encoding='utf-8') as output:
        output.write(line)


def describe_can_error(can_id, data):
    raw = ' '.join('%02X' % b for b in data)
    prefix = 'ID=0x%08X DATA=%s' % (can_id, raw)
    # candleLight v1.0 can_parse_error_status()默认设置CRTL，细节均可为0。
    # 只对这个精确格式作状态提示，不放宽任何带ACK/协议/控制器状态位的帧。
    if can_id == 0x20000004 and list(data) == [0] * 8:
        return 'NOTICE', '空控制器状态通知（未指示具体故障）；' + prefix
    if len(data) != 8:
        return 'ERROR', 'CAN错误帧长度异常；' + prefix
    classes = [(1, '发送超时'), (2, '仲裁丢失'), (4, '控制器状态'),
               (8, '协议错误'), (16, '收发器异常'), (32, '未收到ACK'),
               (64, 'Bus-off总线关闭'), (128, '总线错误'), (256, '控制器重新启动'),
               (512, '错误计数报告')]
    meanings = [name for flag, name in classes if can_id & flag]
    control = [(1, '接收溢出'), (2, '发送溢出'), (4, '接收错误警告'),
               (8, '发送错误警告'), (16, '接收错误被动'), (32, '发送错误被动'),
               (64, '恢复错误主动状态')]
    protocol = [(1, '位错误'), (2, '帧格式错误'), (4, '位填充错误'),
                (8, '显性位发送错误'), (16, '隐性位发送错误'),
                (32, '过载'), (64, '主动错误通知'), (128, '发生在发送阶段')]
    if can_id & 4:
        meanings += [name for flag, name in control if data[1] & flag]
    if can_id & 8:
        meanings += [name for flag, name in protocol if data[2] & flag]
        if data[3] == 8:
            meanings.append('CRC段错误')
    if can_id & 512:
        meanings.append('TX错误计数=%d RX错误计数=%d' % (data[6], data[7]))
    if can_id & 32:
        meanings.append('核对节点供电、CAN H/L、终端和速率')
    return 'ERROR', '；'.join(meanings or ['未识别错误类别']) + '；' + prefix
