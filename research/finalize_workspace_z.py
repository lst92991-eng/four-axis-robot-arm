"""由升降共动及立柱恒定截面证据，完成采样格点XY投影的Z关联推理。"""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'output/constrained-workspace-2026-10-09'
a=json.loads((OUT/'constraint-audit.json').read_text('utf-8'))
p=json.loads((OUT/'z-projection-proof.json').read_text('utf-8'))
r=json.loads((OUT/'z-remaining-evidence.json').read_text('utf-8'))
# 使用整个组件包围盒，不能误取多实体电机的最后一个小实体。
r['moving_component_z_intervals_mm']={c['name']:[c['box'][2]*1000,c['box'][5]*1000]
    for c in a['components'] if c['name'] in ['第二机械臂-1','42步进电机L40-1']}
(OUT/'z-remaining-evidence.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
assert len(r['cases'])==len(p['unresolved'])==6
assert all(c['current_collision_pair']=='第二机械臂-1 / 40x40x500-1' for c in r['cases'])
caps=r['nonvertical_faces']
assert len(caps)==2 and all(c['kind']=='plane' for c in caps)
assert all(abs(c['z_box_mm'][1]-c['z_box_mm'][0])<1e-6 for c in caps)
core=[min(c['z_box_mm'][0] for c in caps),max(c['z_box_mm'][1] for c in caps)]
z=r['moving_component_z_intervals_mm']['第二机械臂-1'];shift=r['z_shift_mm']
swept=[z[0]+shift[0],z[1]+shift[1]]
assert core[0]<swept[0]<swept[1]<core[1]
result={'scope':'4度J1/J2构型采样、按模型连接解释、末端R保持当前相对姿态的腕轴中心XY投影。不是连续路径或全部R姿态验收。',
    'all_z_projection_same_on_sample_grid':True,'carriage_blocked_configurations':p['blocked_at_all_z_by_carriage'],
    'column_blocked_configurations':len(r['cases']),'column_uniform_z_interval_mm':core,
    'second_arm_full_z_sweep_mm':swept,'z_shift_mm':shift,
    'reason':'742组被共同升降的支架阻挡；其余6组第二臂与立柱相交，立柱除上下端面外均为沿Z直母线表面。第二臂全Z扫掠始终处于立柱恒定截面段内，相交不会由升降解除。J1的约±97.74度接触屏障随Z共同升降，不能从直臂分量绕过。当前已通过的3711组在当前高度即提供存在性证据。',
    'not_proven':'绿色XY点在每一个Z高度可达、独立末端R旋转的全部包络、连续无碰撞路径、抑制件/抓手/软管/实物额外挡块。'}
(OUT/'z-projection-conclusion.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
