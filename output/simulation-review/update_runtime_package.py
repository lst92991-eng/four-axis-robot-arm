import os,tarfile
from pathlib import Path
root=Path('C:/Users/lst/Desktop/scara')
archive=root/'output/release/scara-cad-sim-2026-10-10.tar.gz'
temporary=archive.with_name('scara-cad-sim-2026-10-10.next.tar.gz')
names=['sim.cpp','chess/pikafish.cpp','chess/pikafish.hpp','README.md','start-gpu.ps1']
replacements={'scara-sim/'+name:root/'simulation'/name for name in names}
replacements['scara-sim/build/scara_sim']=root/'output/release/scara-cad-sim-2026-10-10/scara_sim'
replacements['scara-sim/docs/X42S驱动分析与实现报告.md']=root/'docs/X42S驱动分析与实现报告.md'
for name in ('engine-gui-wait-before.log','engine-gui-wait-after.log'):
    replacements['scara-sim/output/validation/'+name]=root/'output/simulation-review'/name
with tarfile.open(archive,'r:gz') as source,tarfile.open(temporary,'w:gz') as output:
    seen=set()
    for member in source:
        if member.name in replacements:
            output.add(replacements[member.name],arcname=member.name)
            seen.add(member.name)
        else:
            output.addfile(member,source.extractfile(member) if member.isfile() else None)
    for name in replacements.keys()-seen:
        output.add(replacements[name],arcname=name)
backup=archive.with_name('scara-cad-sim-2026-10-10-before-engine-fix.tar.gz')
if not backup.exists(): archive.rename(backup)
os.replace(temporary,archive)
print('运行包已更新；原包保留为before-engine-fix。')
