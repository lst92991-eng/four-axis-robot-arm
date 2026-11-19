"""Check the saved-data boundary; production decoding receives validated tensor views."""
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
packet = (ROOT/'tests/fixtures/vision/rknn-output.bin').read_bytes()
header_size = struct.unpack_from('<I',packet)[0]
original = json.loads(packet[4:4+header_size])
payload = packet[4+header_size:]
directory = ROOT/'output/vision-stage-verify'
directory.mkdir(parents=True,exist_ok=True)

with tempfile.TemporaryDirectory(prefix='tensor-input-',dir=directory) as temporary:
    folder = Path(temporary).resolve()
    assert folder.is_relative_to(directory.resolve())
    for name in ('scale','format','stride','dimensions','truncated'):
        meta = json.loads(json.dumps(original))
        attr = meta['outputs'][0]
        if name=='scale': attr['scale'] = -1
        if name=='format': attr['fmt'] = 0
        if name=='stride': attr['w_stride'] = 81
        if name=='dimensions': attr['dims'][3] = 58
        header = json.dumps(meta).encode()
        binary = folder/(name+'.bin')
        binary.write_bytes(struct.pack('<I',len(header))+header+(payload[:-1] if name=='truncated' else payload))
        manifest = folder/(name+'.json')
        manifest.write_text(json.dumps({'cases':[dict(id=name,image='tests/fixtures/vision/source.png',
            packet=binary.relative_to(ROOT).as_posix(),roi=[300,100,620,600])]}),encoding='utf-8')
        output = folder/(name+'-output')
        result = subprocess.run([sys.argv[1],str(manifest.relative_to(ROOT)),str(output.relative_to(ROOT))],
                                cwd=ROOT,capture_output=True,text=True,timeout=15)
        expected = 'Truncated tensor data' if name=='truncated' else 'Unexpected recorded tensor layout'
        assert result.returncode==1 and expected in result.stderr,(name,result.stdout,result.stderr)
        assert not (output/(name+'.json')).exists(),name
print('PASS invalid saved tensor scale, format, stride, dimensions and truncated payload')
