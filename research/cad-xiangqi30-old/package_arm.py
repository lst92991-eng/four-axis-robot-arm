import sys,json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
SKILL=Path('C:/Users/lst/.codex/tools/solidworks-automation-mcp')
sys.path.insert(0,str(SKILL/'scripts'))
from sw_delivery import _comtypes_module
_comtypes_module()
import comtypes.client
from comtypes.gen import SldWorks
before=json.loads((ROOT/'research/cad-xiangqi30-old/session-before.json').read_text('utf-8'))
source=Path(before['source']['path'])
sw=comtypes.client.GetActiveObject('SldWorks.Application').QueryInterface(SldWorks.ISldWorks)
doc=sw.GetOpenDocumentByName(str(source)).QueryInterface(SldWorks.IModelDoc2)
if doc.GetSaveFlag(): raise RuntimeError('Original assembly has unsaved changes')
extension=doc.Extension.QueryInterface(SldWorks.IModelDocExtension)
package=extension.GetPackAndGo()
package.IncludeDrawings=False
package.IncludeToolboxComponents=True
package.IncludeSuppressed=True
package.FlattenToSingleFolder=True
package.AddPrefix='OLD_'
target=ROOT/'hardware/xiangqi30-old195275-2026-10-12/native/arm'
target.mkdir(parents=True,exist_ok=True)
assert not list(target.glob('*.SLDASM')),'Do not overwrite an existing package'
count=package.GetDocumentNamesCount()
print('Native Pack and Go documents:',count,flush=True)
assert package.SetSaveToName(True,str(target)+'\\')
codes=extension.SavePackAndGo(package)
report={'count':count,'codes':list(codes or []),'files':[p.name for p in target.iterdir()],
        'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'source_dirty':doc.GetSaveFlag()}
(ROOT/'research/cad-xiangqi30-old/pack-report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='files'},ensure_ascii=False),flush=True)
assert not report['source_dirty'] and report['source_sha256']==before['source']['sha256']
assert report['files'] and all(v==0 for v in report['codes'])
