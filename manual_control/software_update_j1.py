"""Update only J1 while stopped, preserving its logical multi-turn reference."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

from calibrate_j1 import Session
from stepfoc_candlelight import Frame
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'temp'))
from ota import unpack,send_image


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--from-build',required=True)
    p.add_argument('--image',type=Path,required=True)
    args=p.parse_args();manifest=json.loads(args.image.with_name('J1-manifest.json').read_text())
    data=args.image.read_bytes();node,crc,img=unpack(data)
    assert node==1 and manifest['node']==1
    assert hashlib.sha256(data).hexdigest()==manifest['files'][args.image.name]
    root=Path(__file__).resolve().parents[1]/'output/j1-bench-20261109'/('update-'+manifest['build_id'])
    record={'manifest':manifest};s=Session('37813482333551062',root/'before')
    try:
        initial=s.wait(lambda v:v['stationary'],3);record['initial']=dict(initial)
        assert initial['build']==args.from_build and initial['encoder_valid'] and initial['stored']
        s.sync()
        if initial['power']:s.command(5)
        s.wait(lambda v:not v['power'] and v['stationary'],2,True)
        end=time.monotonic()+.3
        while time.monotonic()<end:s.receive(.005)
        before=dict(s.status);record['before']=before
        assert abs(before['position']-initial['position'])<1
        if before['fault']:
            s.command(6);s.wait(lambda v:v['state']==0 and not v['fault'] and v['encoder_valid'],2,True)
        s.command(11,0x52454254)
        def send(m,timeout=None):s.adapter.send(Frame(m.arbitration_id,m.data,fd=True,brs=True))
        def recv(timeout):
            f=s.adapter.recv(timeout)
            if f and not f.echo:return SimpleNamespace(arbitration_id=f.identifier,data=f.data,is_error_frame=f.error,is_extended_id=f.extended,is_fd=f.fd)
        send_image(SimpleNamespace(send=send,recv=recv),SimpleNamespace(Message=SimpleNamespace),1,crc,img,25)
    finally:s.close()
    s=Session('37813482333551062',root/'after')
    try:
        after=s.wait(lambda v:True,3)
        assert after['build']==manifest['build_id'] and after['stored'] and not after['power'] and not after['fault']
        assert (after['raw_zero'],after['direction'])==(before['raw_zero'],before['direction'])
        delta=((after['counts']-(before['counts']&16383)+8192)&16383)-8192
        assert abs(delta)<100
        logical=before['position']+delta*360/16384
        s.sync();s.command(2,round(logical*1000))
        record['after']=s.wait(lambda v:v['referenced'] and v['stationary'],2,True)
        print('UPDATED '+json.dumps(record['after']),flush=True)
    finally:
        s.close();(root/'record.json').write_text(json.dumps(record,indent=2)+'\n')


if __name__=='__main__':main()
