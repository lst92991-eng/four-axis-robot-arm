import json,hashlib,sys
from pathlib import Path
import win32com.client
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import get_com_member as get

sw=win32com.client.GetActiveObject('SldWorks.Application')
doc=sw.ActiveDoc
result={'revision':get(sw,'RevisionNumber'),'active':None}
if doc:
    result['active']={'path':get(doc,'GetPathName'),'title':get(doc,'GetTitle'),'dirty':get(doc,'GetSaveFlag')}
model_path=Path(json.loads(Path('simulation/assets/arm/manifest.json').read_text(encoding='utf-8'))['assembly'])
result['source']={'path':str(model_path),'sha256':hashlib.sha256(model_path.read_bytes()).hexdigest()}
Path('research/cad-xiangqi30-old/session-before.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
