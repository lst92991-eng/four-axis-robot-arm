"""本次已由用户确认：仅末端电机、地址1、卸下负载。只校准一次，不使能/回零/改ID。"""
import json,struct,time
from pathlib import Path
from datetime import datetime,timezone,timedelta
import usb.core,usb.util,usb.backend.libusb1,libusb_package
from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame

stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d-%H%M%S')
path=Path(__file__).resolve().parent/('wrist-calibration-'+stamp+'.json')
report={'time':stamp,'physical_axis':'isolated unloaded wrist motor, confirmed by user','node':1,'bitrate':500000,'frames':[],'samples':[],'calibration_commands_sent':0,'success':False}
backend=usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
devices=list(usb.core.find(find_all=True,idVendor=0x1D50,idProduct=0x606F,backend=backend))
assert len(devices)==1,'Expected one USB-CAN'
dev=devices[0];assert dev.serial_number=='004A00404346570B20363833'
can=GsUsb(dev)
def record(direction,f):
    report['frames'].append({'time':time.time(),'direction':direction,'id':hex(f.can_id),'echo':f.echo_id,'data':list(f.data[:f.can_dlc])})
def exchange(data,timeout=.4):
    assert data==[6,0x45,0x6B] or (len(data)==2 and data[0] in (0x1A,0x3B,0x3A,0x24,0x35,0x27) and data[1]==0x6B)
    tx=GsUsbFrame(can_id=0x80000100,data=data);can.send(tx);record('tx',tx)
    if data[0]==6:report['calibration_commands_sent']+=1
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        f=GsUsbFrame()
        if not can.read(f,10):continue
        record('rx',f)
        if f.echo_id!=0xFFFFFFFF:continue
        b=list(f.data[:f.can_dlc])
        if f.can_id&0x20000000:
            if f.can_id==0x20000004 and b==[0]*8:continue
            raise RuntimeError('CAN error '+hex(f.can_id)+' '+str(b))
        if f.can_id==0x80000100 and b and b[0]==data[0]:
            assert b[-1]==0x6B
            return b
    return None
def query(code):return exchange([code,0x6B])
try:
    can.stop();dev.ctrl_transfer(0x41,0,0,0,struct.pack('<I',0xBEEF))
    assert can.set_bitrate(500000)
    can.device_flags=0;dev.ctrl_transfer(0x41,2,0,0,struct.pack('<II',1,0))
    until=time.monotonic()+.3
    while time.monotonic()<until:
        f=GsUsbFrame()
        if can.read(f,10):record('startup_rx',f)
    mode=query(0x1A);flags=query(0x3B);rpm=query(0x35)
    report['before']={'mode':mode,'flags':flags,'rpm':rpm,'status':query(0x3A),'voltage':query(0x24)}
    assert mode==[26,0,6,107],'Emm mode changed; no calibration sent'
    assert flags and len(flags)==3 and flags[1]&0x31==1,'Encoder not ready or protection active'
    assert rpm and len(rpm)==5 and rpm[2:4]==[0,0],'Motor not stationary'
    if flags[1]&2:
        report['success']=True;report['result']='Already calibrated; no repeated command'
    else:
        ack=exchange([6,0x45,0x6B],1)
        report['calibration_ack']=ack
        print('Calibration command sent once; ACK='+str(ack),flush=True)
        assert ack==[6,2,107],'Calibration ACK missing/rejected; no retry'
        started=time.monotonic();stable=0;next_progress=0
        while time.monotonic()-started<60:
            flags=query(0x3B)
            sample={'elapsed_s':round(time.monotonic()-started,3),'flags':flags}
            if flags:
                assert len(flags)==3 and flags[1]&0x30==0,'Driver overtemperature/overcurrent protection'
                if flags[1]&3==3:
                    rpm=query(0x35);sample['rpm']=rpm
                    stable=stable+1 if rpm and len(rpm)==5 and rpm[2:4]==[0,0] else 0
                else:stable=0
            else:stable=0
            report['samples'].append(sample)
            if stable>=3:
                report['success']=True;report['result']='Encoder ready + calibration ready + three stationary samples'
                report['elapsed_seconds']=time.monotonic()-started;break
            if time.monotonic()-started>=next_progress:
                print('Waiting',sample,flush=True);next_progress+=5
            time.sleep(.5)
        assert report['success'],'Calibration completion not confirmed within 60 seconds'
    report['after']={'mode':query(0x1A),'flags':query(0x3B),'rpm':query(0x35),'status':query(0x3A),'voltage':query(0x24),'phase_current':query(0x27)}
    assert report['after']['flags'] and report['after']['flags'][1]&0x33==3
    assert report['after']['rpm'] and report['after']['rpm'][2:4]==[0,0]
except Exception as error:
    report['success']=False;report['error']=str(error)
finally:
    can.stop();usb.util.dispose_resources(dev)
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf-8')
print(json.dumps({'report':str(path),'success':report['success'],'calibration_commands_sent':report['calibration_commands_sent'],'before':report.get('before'),'after':report.get('after'),'error':report.get('error')},ensure_ascii=False),flush=True)
