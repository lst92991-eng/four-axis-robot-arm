"""只读原装配，原生Pack and Go独立重命名副本；不保存原CAD。"""
import sys,json,hashlib
from pathlib import Path
import win32com.client

SKILL=Path('C:/Users/lst/.codex/tools/solidworks-automation-mcp')
sys.path.insert(0,str(SKILL/'scripts'))
from sw_connect import get_com_member as get
from sw_delivery import _get_pack_and_go
sys.path.insert(0,str(SKILL/'apps/desktop/cad_workbench'))
from engineering_orchestrator import build_engineering_plan

OUT=Path('hardware/xiangqi30-old195275-2026-10-12').resolve()
OUT.mkdir(parents=True,exist_ok=True)
plan=build_engineering_plan('旧版195+275mm机械臂参考总装：新建30mm中国象棋及棋盘、四个ArUco打印标记，原生装配并打包STEP/预览；不改原机，不设计制造相机架。')
(OUT/'execution-plan.json').write_text(json.dumps(plan.to_dict(),ensure_ascii=False,indent=2),encoding='utf-8')
before=json.loads(Path('research/cad-xiangqi30-old/session-before.json').read_text(encoding='utf-8'))
source=Path(before['source']['path'])
sw=win32com.client.GetActiveObject('SldWorks.Application')
doc=sw.GetOpenDocumentByName(str(source))
if not doc or get(doc,'GetSaveFlag'):
    raise RuntimeError('原CAD不存在于会话或有未保存更改，不据此自动生成副本')
target=OUT/'native/arm'
target.mkdir(parents=True,exist_ok=True)
if list(target.glob('*.SLDASM')): raise RuntimeError('副本已存在，不重复覆盖Pack and Go')
pg=_get_pack_and_go(doc.Extension)
pg.IncludeDrawings=False
pg.IncludeSuppressed=True
pg.IncludeToolboxComponents=True
pg.FlattenToSingleFolder=True
pg.AddPrefix='OLD195275_'
count=int(get(pg,'GetDocumentNamesCount'))
if not pg.SetSaveToName(True,str(target)+'\\'): raise RuntimeError('Pack and Go目标设置失败')
codes=doc.Extension.SavePackAndGo(pg)
record={'source':str(source),'source_sha256_before':before['source']['sha256'],
        'source_sha256_after':hashlib.sha256(source.read_bytes()).hexdigest(),
        'source_dirty_after':bool(get(doc,'GetSaveFlag')),'documents':count,
        'status_codes':codes,'outputs':[p.name for p in target.iterdir()]}
(OUT/'pack-and-go-evidence.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
if record['source_sha256_before']!=record['source_sha256_after'] or record['source_dirty_after']:
    raise RuntimeError('原文件保护检查失败')
print(json.dumps({'documents':count,'copied_files':len(record['outputs']),'codes':codes},ensure_ascii=False))
