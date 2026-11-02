import hashlib,json,subprocess
from pathlib import Path
root=Path('D:/BaiduNetdiskDownload/ZDT_XS系列第二代闭环步进电机资料')
archive=next((root/'14.例程_TI_MSPM0G3507/X固件模式/串口通讯').glob('*梯形曲线加减速位置模式控制.rar'))
dest=Path('research/mechanism/driver-x42s/vendor-review')
dest.mkdir(exist_ok=True)
records={'archive':str(archive),'archive_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'files':{}}
for name in ('X_V2.c','X_V2.h'):
    data=subprocess.check_output(['tar','-xOf',str(archive),'--include','*/bsp/'+name])
    (dest/name.replace('.','-decoded.')).write_text(data.decode('gb18030',errors='replace'),encoding='utf-8')
    records['files'][name]={'sha256_original':hashlib.sha256(data).hexdigest(),'bytes':len(data)}
(dest/'provenance.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
can_archive=next((root/'10.例程_STM32F103/HAL库/X固件模式/CAN通讯').glob('*梯形曲线加减速位置模式控制.rar'))
can_record={'archive':str(can_archive),'archive_sha256':hashlib.sha256(can_archive.read_bytes()).hexdigest(),'files':{}}
for name in ('X_V2.c','X_V2.h','can.c','main.c'):
    member='*/BSP/'+name if name.startswith('X_') else '*/Core/Src/'+name
    data=subprocess.check_output(['tar','-xOf',str(can_archive),'--include',member])
    (dest/('can-'+name.replace('.','-decoded.'))).write_text(data.decode('gb18030',errors='replace'),encoding='utf-8')
    can_record['files'][name]={'sha256_original':hashlib.sha256(data).hexdigest(),'bytes':len(data)}
(dest/'can-provenance.json').write_text(json.dumps(can_record,ensure_ascii=False,indent=2),encoding='utf-8')
f407=next((root/'9. 例程_STM32F407/HAL库/X固件模式/CAN通讯').glob('*梯形曲线加减速位置模式控制.rar'))
record407={'archive':str(f407),'sha256':hashlib.sha256(f407.read_bytes()).hexdigest(),'files':{}}
for name in ('X_V2.c','can.c','main.c'):
    member='*/Src/'+name
    data=subprocess.check_output(['tar','-xOf',str(f407),'--include',member])
    (dest/('f407-'+name.replace('.','-decoded.'))).write_text(data.decode('gb18030',errors='replace'),encoding='utf-8')
    record407['files'][name]=hashlib.sha256(data).hexdigest()
(dest/'f407-provenance.json').write_text(json.dumps(record407,ensure_ascii=False,indent=2),encoding='utf-8')
