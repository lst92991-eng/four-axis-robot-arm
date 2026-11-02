"""原生Pack and Go独立复制整机；复制前后核对原文件哈希。"""
import sys,json,hashlib,os
from pathlib import Path
import pythoncom,win32com.client
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'research'))
from probe_cad_collision import SW,DOC,get
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp')
from scripts.sw_delivery import _get_pack_and_go
OUT=ROOT/'hardware/arm_equal_275/v1'
NATIVE=OUT/'native'

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def main():
    OUT.mkdir(parents=True,exist_ok=True)
    if NATIVE.exists() and any(NATIVE.iterdir()):raise RuntimeError('复制目标非空，拒绝覆盖')
    NATIVE.mkdir(exist_ok=True)
    source=get(DOC,'GetPathName');before=get(DOC,'GetSaveFlag');paths={source}
    for c in DOC.GetComponents(False):
        path=get(c,'GetPathName')
        if path and Path(path).is_file():paths.add(path)
    source_hashes={p:digest(p) for p in paths}
    (OUT/'source-before-copy.json').write_text(json.dumps({'source':source,'save_flag':before,'sha256':source_hashes},ensure_ascii=False,indent=2),encoding='utf-8')
    # 当前2026版本的Pack and Go早绑定接口可用，避免晚绑定参数封送问题。
    import comtypes.client
    comtypes.client.GetModule(('{83A33D31-27C5-11CE-BFD4-00400513BB57}',34,0))
    from comtypes.gen import SldWorks
    typed_sw=comtypes.client.GetActiveObject('SldWorks.Application',interface=SldWorks.ISldWorks)
    if typed_sw.GetProcessID()!=get(SW,'GetProcessID'):raise RuntimeError('SolidWorks实例不一致')
    previous=get(get(SW,'ActiveDoc'),'GetTitle')
    typed_doc=typed_sw.GetOpenDocumentByName(source).QueryInterface(SldWorks.IModelDoc2)
    typed_sw.ActivateDoc3(typed_doc.GetTitle(),False,0)
    ext=typed_doc.Extension.QueryInterface(SldWorks.IModelDocExtension);pg=ext.GetPackAndGo()
    pg.IncludeDrawings=False;pg.IncludeSimulationResults=False;pg.IncludeToolboxComponents=True
    pg.IncludeSuppressed=True;pg.FlattenToSingleFolder=True
    pg.AddPrefix='EQ275_'
    if not pg.SetSaveToName(True,str(NATIVE)+os.sep):raise RuntimeError('Pack and Go target rejected')
    count=pg.GetDocumentNamesCount()
    print('Pack and Go documents',count,flush=True)
    statuses=ext.SavePackAndGo(pg)
    typed_sw.ActivateDoc3(previous,False,0)
    changed=[p for p,h in source_hashes.items() if digest(p)!=h]
    report={'source':source,'target':str(NATIVE),'reported_count':count,'status_codes':list(statuses) if statuses is not None else None,
        'save_flag_before':before,'save_flag_after':get(DOC,'GetSaveFlag'),'source_files_changed':changed,
        'outputs':[str(p.relative_to(OUT)) for p in NATIVE.rglob('*') if p.is_file()]}
    (OUT/'copy-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False),flush=True)
    if changed:raise RuntimeError('原文件发生变化，停止后续编辑')
    if not list(NATIVE.glob('*.SLDASM')):raise RuntimeError('未生成装配副本')

if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
