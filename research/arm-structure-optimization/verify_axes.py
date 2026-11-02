"""回读临时实体圆柱面轴线，确认改长没有只改外形而漏改关节孔位。"""
import json,sys
import numpy as np
from compare_cad_variants import Factory,OUT,S,get,DOC
def axes(b):
    result=[]
    for face in get(b,'GetFaces'):
        surf=get(face,'GetSurface')
        if get(surf,'IsCylinder'):
            p=surf.CylinderParams
            if abs(abs(p[5])-1)<1e-8:
                xy=(-(np.array(p[:2])-S[:2])*1000).tolist()
                if not any(np.linalg.norm(np.array(xy)-v)<1e-5 for v in result):result.append(xy)
    return result
if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8');f=Factory();results=[]
    for label,args in [('ratio_225_245',{'d1':30,'d2':-30}),('first_235_275',{'d1':40})]:
        scene=f.scene(label,**args);record={'label':label,'parts':{}}
        for name,expected in [('J1机械臂-2',[[0,0],[0,scene.l1]]),('第二机械臂-1',[[0,scene.l1],[0,scene.l1+scene.l2]])]:
            source=next(r for r in scene.rs if r['name']==name);found=axes(source['body'])
            errors=[min(np.linalg.norm(np.array(v)-e) for v in found) for e in expected]
            assert max(errors)<1e-4
            record['parts'][name]={'expected_axes_mm':expected,'axis_match_error_mm':errors,'detected_vertical_axes_mm':found}
        results.append(record)
    output={'method':'改型后的SolidWorks临时实体圆柱面轴线回读','results':results,'source_save_flag_before':f.before,'source_save_flag_after':get(DOC,'GetSaveFlag')}
    (OUT/'modified-joint-axis-validation.json').write_text(json.dumps(output,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps([{ 'label':r['label'],'axis_checks':{k:v['axis_match_error_mm'] for k,v in r['parts'].items()} } for r in results],ensure_ascii=False))
