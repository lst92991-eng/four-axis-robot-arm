"""肘轴接入只读核对：仅查询1F/1A/3B/3A/24/35，不使能、不运动、不改驱动。"""
import json,struct,time
from pathlib import Path
from datetime import datetime,timezone,timedelta
import usb.core,usb.util,usb.backend.libusb1,libusb_package
from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame

root=Path(__file__).resolve().parent
stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d-%H%M%S')
report={'time':stamp,'purpose':'read_only_elbow_identification','bitrate':500000,'frames':[],'nodes':{}}
path=root/('elbow-readonly-'+stamp+'.json')
backend=usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
devices=list(usb.core.find(find_all=True,idVendor=0x1D50,idProduct=0x606F,backend=backend))
assert len(devices)==1,'Expected one USB-CAN'
dev=devices[0];assert dev.serial_number=='004A00404346570B20363833','USB-CAN identity changed'
can=GsUsb(dev)
def record(direction,f):
    report['frames'].append({'time':time.time(),'direction':direction,'id':hex(f.can_id),'echo':f.echo_id,'data':list(f.data[:f.can_dlc])})
def query(node,code,timeout=.3):
    assert 1<=node<=4 and code in (0x1F,0x1A,0x3B,0x3A,0x24,0x35)
    tx=GsUsbFrame(can_id=0x80000000|(node<<8),data=[code,0x6B]);can.send(tx);record('tx',tx)
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        f=GsUsbFrame()
        if not can.read(f,10):continue
        record('rx',f)
        if f.echo_id!=0xFFFFFFFF:continue
        data=list(f.data[:f.can_dlc])
        if f.can_id&0x20000000:
            if f.can_id==0x20000004 and data==[0]*8:continue
            raise RuntimeError('CAN error '+hex(f.can_id)+' '+str(data))
        if f.can_id==(0x80000000|(node<<8)) and data and data[0]==code:
            assert data[-1]==0x6B,'Unexpected checksum setting'
            return data
    return None
try:
    can.stop();dev.ctrl_transfer(0x41,0,0,0,struct.pack('<I',0xBEEF))
    assert can.set_bitrate(500000)
    can.device_flags=0;dev.ctrl_transfer(0x41,2,0,0,struct.pack('<II',1,0))
    end=time.monotonic()+.3
    while time.monotonic()<end:
        f=GsUsbFrame()
        if can.read(f,10):record('startup_rx',f)
    for node in (3,1,2,4):
        data=query(node,0x1F)
        if data is None:continue
        result={'version':data}
        for code in (0x1A,0x3B,0x3A,0x24,0x35):result[hex(code)]=query(node,code)
        report['nodes'][str(node)]=result
except Exception as error:
    report['error']=str(error)
finally:
    can.stop();usb.util.dispose_resources(dev)
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf-8')
print(json.dumps({'report':str(path),'nodes':report['nodes'],'error':report.get('error')},ensure_ascii=False))
