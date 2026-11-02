"""保留30mm交付；复制两版总装并把14种棋子圆饼的驱动直径改33mm。"""
import sys,json,shutil,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'hardware/xiangqi33-assemblies-2026-10-13';OUT.mkdir(exist_ok=True)
OLD=ROOT/'hardware/xiangqi30-old195275-2026-10-12/delivery-front-2026-10-13'
NEW=ROOT/'hardware/xiangqi30-long275275-2026-10-13'
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g,open_document,save_document
sw=win32com_client.GetActiveObject('SldWorks.Application');sw=win32com_client.dynamic.DumbDispatch(sw._oleobj_,'SldWorks.Application')
baseline={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for folder in (OLD/'SolidWorks',NEW/'SolidWorks') for p in folder.iterdir() if p.suffix.lower() in ('.sldasm','.sldprt') and not p.name.startswith('~$')}
(OUT/'source-30mm-sha256.json').write_text(json.dumps(baseline,ensure_ascii=False,indent=2),'utf-8')
stages=[]
for label,src,prefix,links in [('old',OLD,'O33_',195),('long',NEW,'N33_',275)]:
    dest=OUT/label/'SolidWorks';dest.mkdir(parents=True,exist_ok=True)
    mapping={};source_main=src/f'SolidWorks/SCARA_FRONT_{links}_275_CHESS30.SLDASM'
    for p in (src/'SolidWorks').iterdir():
        if p.name.startswith('~$') or p.suffix.lower() not in ('.sldasm','.sldprt'):continue
        name=f'SCARA_FRONT_{links}_275_CHESS33.SLDASM' if p==source_main else prefix+p.name
        target=dest/name;assert not target.exists(),'Do not overwrite an existing 33mm stage'
        shutil.copy2(p,target);mapping[p.name]=target
    rewrites=[]
    for p in dest.glob('*.SLDASM'):
        for prior in list(sw.GetDocumentDependencies2(str(p),False,False,False) or [])[1::2]:
            assert '^' not in Path(prior).name
            target=mapping[Path(prior).name]
            assert sw.ReplaceReferencedDocument(str(p),prior,str(target)),(p,prior)
            rewrites.append({'assembly':p.name,'old':prior,'new':str(target)})
    pieces=[p for p in dest.glob('*.SLDPRT') if 'piece_' in p.name];assert len(pieces)==14
    dimensions=[]
    for path in pieces:
        model=open_document(sw,str(path),silent=True,raise_on_error=True)
        sketch=model.FeatureByName('草图1');assert sketch is not None
        display=g(sketch,'GetFirstDisplayDimension');dimension=display.GetDimension2(0)
        assert abs(g(dimension,'SystemValue')-.03)<1e-8
        dimension.SystemValue=.033
        feature=model.FeatureByName('Diameter30_Height8');assert feature is not None;feature.Name='Diameter33_Height8'
        assert model.ForceRebuild3(False)
        bodies=model.GetBodies2(0,False);bb=list(g(bodies[0],'GetBodyBox'));size=[(bb[i+3]-bb[i])*1000 for i in range(3)]
        assert abs(size[0]-33)<1e-4 and abs(size[1]-33)<1e-4,(path,size)
        assert abs(g(dimension,'SystemValue')-.033)<1e-8
        assert save_document(model)
        dimensions.append({'file':path.name,'diameter_mm':33,'bbox_size_mm':size,'driving_dimension':g(dimension,'FullName')})
        sw.CloseDoc(g(model,'GetTitle'));print('33mm',label,path.name,flush=True)
    main=dest/f'SCARA_FRONT_{links}_275_CHESS33.SLDASM'
    deps=list(sw.GetDocumentDependencies2(str(main),True,False,False) or [])[1::2]
    assert all(Path(p).is_relative_to(dest) and Path(p).exists() for p in deps)
    layout=json.loads((src/'assembly-layout.json').read_text('utf-8'))
    layout['assembly']=str(main);layout['arm_links_mm']=[links,275];layout['board']['piece_diameter_mm']=33
    layout['board']['nominal_piece_gap_mm']=2
    for row in layout['components']:
        row['file']=mapping[Path(row['file']).name].name
    layout['board'].pop('old_collision_grid_supported_points',None)
    stage={'label':label,'source':str(src),'main':str(main),'links_mm':[links,275],'prefix':prefix,'dimensions':dimensions,'reference_rewrites':rewrites}
    (OUT/label/'assembly-layout.json').write_text(json.dumps(layout,ensure_ascii=False,indent=2),'utf-8')
    (OUT/label/'piece-dimensions.json').write_text(json.dumps(dimensions,ensure_ascii=False,indent=2),'utf-8')
    stages.append(stage)
for label in ('old','long'):
    shutil.copytree(OLD/'markers',OUT/label/'markers',dirs_exist_ok=True)
shutil.copytree(OLD/'reference',OUT/'reference',dirs_exist_ok=True)
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in baseline.items())
(OUT/'stages.json').write_text(json.dumps(stages,ensure_ascii=False,indent=2),'utf-8')
print('BOTH 33mm native packages prepared; 30mm source hashes unchanged',flush=True)
