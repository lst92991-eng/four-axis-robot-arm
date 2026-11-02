"""走棋时间的条件化计算；不读取或控制实物，不改运行参数。"""
import json,math,itertools
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output/move-time-estimate-2026-10-09';OUT.mkdir(parents=True,exist_ok=True)
grid=np.load(ROOT/'output/constrained-workspace-2026-10-09/cad-collision-scan.npz')
angles=grid['angles'];free=grid['connected'];step=float(angles[1]-angles[0])
LIFT_MM=20  # 用户指定的计时方案；表示从抓取接触位置向上移动20mm。
THINK_BUDGET_S=json.loads((ROOT/'config/robot.json').read_text('utf-8'))['think_ms']/1000
ENGINE_EVIDENCE='output/rv1106/pikafish-validation/2023-03-05/board-results.json'
engine_bench=json.loads((ROOT/ENGINE_EVIDENCE).read_text('utf-8'))
profiles=[
    {'name':'较慢假设','j1_motor_rpm':300,'j1_motor_acc_rpm_s':600,'joint_v_deg_s':36,'joint_a_deg_s2':72,'z_v_mm_s':20,'z_a_mm_s2':80},
    {'name':'中档假设','j1_motor_rpm':600,'j1_motor_acc_rpm_s':1200,'joint_v_deg_s':72,'joint_a_deg_s2':144,'z_v_mm_s':40,'z_a_mm_s2':160},
    {'name':'较快假设','j1_motor_rpm':900,'j1_motor_acc_rpm_s':1800,'joint_v_deg_s':108,'joint_a_deg_s2':216,'z_v_mm_s':60,'z_a_mm_s2':240},
]
def duration(s,v,a):
    if s<1e-12:return 0.0
    return 2*math.sqrt(s/a) if s<=v*v/a else s/v+v/a
def ik(xy,sign):
    x,y=xy;c=(x*x+y*y-195**2-275**2)/(2*195*275)
    if not -1<=c<=1:return None
    q2=sign*math.acos(c)
    q1=math.atan2(y,x)-math.atan2(275*math.sin(q2),195+275*math.cos(q2))-math.pi/2
    return np.array([(math.degrees(q1)+180)%360-180,math.degrees(q2)])
def supported(q):
    u=(q-angles[0])/step
    i,j=np.floor(u).astype(int)
    if not 0<=i<len(angles)-1 or not 0<=j<len(angles)-1:return False
    return bool(free[i:i+2,j:j+2].all())
def path_supported(a,b):
    n=max(2,int(math.ceil(np.max(abs(b-a))/.5))+1)
    return all(supported(a+t*(b-a)) for t in np.linspace(0,1,n)),n
W=(0,440);C=(270,210)
cases=[
    {'name':'相邻交点40mm','a':(20,256),'b':(-20,256),'capture':False},
    {'name':'沿棋盘200mm','a':(-100,296),'b':(100,296),'capture':False},
    {'name':'横跨长边360mm','a':(-180,336),'b':(180,336),'capture':False},
    {'name':'横跨360mm并吃子','a':(-180,336),'b':(180,336),'capture':True},
]
result={'confirmed':{'j1_ratio':50,'L1_mm':195,'L2_mm':275},'assumptions':{'j2_ratio':3,'lift_height_mm':LIFT_MM,'lift_source':'用户指定20mm计时方案，未实测通过净空','suction_s':.5,'release_s':.3,'communication_and_settling_allowance_per_transfer_s':.2,'wait_xy_mm':W,'capture_bin_xy_mm':C,'r_axis':'保持当前相对第二臂姿态，不独立旋转棋子'},'thinking':{'configured_search_budget_s':THINK_BUDGET_S,'source':'config/robot.json think_ms；chess/pikafish.cpp go movetime','scope':'仅搜索预算，不是完整思考阶段实测值；不含引擎启动/加载、合法性检查、退出、视觉识别'},'profiles':profiles,'cases':[]}
result['thinking'].update(reference_measurement_source=ENGINE_EVIDENCE,
    reference_measured_search_s=[s['elapsed_seconds'] for s in engine_bench['searches'] if s['requested_ms']==1000],
    reference_measured_startup_s=engine_bench['startup_seconds'],
    startup_policy='当前main.cpp思考分支调用engine.start并在搜索后close；启动耗时应单列。历史独立引擎测试不等于当前完整视觉联动时延。')
result['perception']={'total_seconds':None,'note':'采集、稳定帧确认、YOLO推理、棋局校验与落子后复核单独计时；不能用单次NPU推理时间代替完整视觉链路。'}
for p in profiles:
    assert abs(p['j1_motor_rpm']*6/50-p['joint_v_deg_s'])<1e-8
    assert abs(p['j1_motor_acc_rpm_s']*6/50-p['joint_a_deg_s2'])<1e-8
    p['j2_motor_rpm_assuming_3_to_1']=p['joint_v_deg_s']*3/6
    p['j2_motor_acc_rpm_s_assuming_3_to_1']=p['joint_a_deg_s2']*3/6
for case in cases:
    route=[W,case['b'],C,case['a'],case['b'],W] if case['capture'] else [W,case['a'],case['b'],W]
    branches=[]
    for sign in [-1,1]:
        qs=[ik(pt,sign) for pt in route]
        if any(q is None for q in qs):continue
        checks=[path_supported(a,b) for a,b in zip(qs,qs[1:])]
        if not all(c[0] for c in checks):continue
        distances=[np.abs(b-a).tolist() for a,b in zip(qs,qs[1:])]
        # 两轴采用同一归一化进度曲线，保证沿已筛查的关节直线路径运动。
        # 两轴速度/加速度限值相同，主导位移为max(|dq1|,|dq2|)。
        totals=[sum(duration(max(d),p['joint_v_deg_s'],p['joint_a_deg_s2']) for d in distances) for p in profiles]
        branches.append({'elbow_sign':sign,'joints_deg':[q.tolist() for q in qs],'delta_deg':distances,'horizontal_totals_s':totals,'path_sample_count':sum(c[1] for c in checks)})
    if not branches:raise RuntimeError('No grid-supported same-elbow route for '+case['name'])
    branch=min(branches,key=lambda b:b['horizontal_totals_s'][1])
    transfers=2 if case['capture'] else 1
    timings=[]
    for index,p in enumerate(profiles):
        vertical=4*transfers*duration(LIFT_MM,p['z_v_mm_s'],p['z_a_mm_s2'])
        dwell=transfers*(.5+.3+.2)
        horizontal=branch['horizontal_totals_s'][index]
        carrying=duration(max(branch['delta_deg'][-2] if case['capture'] else branch['delta_deg'][1]),p['joint_v_deg_s'],p['joint_a_deg_s2'])
        mechanical=horizontal+vertical+dwell
        timings.append({'profile':p['name'],'horizontal_all_legs_s':horizontal,'vertical_s':vertical,
            'mechanical_motion_s':horizontal+vertical,'suction_release_s':transfers*.8,
            'communication_settling_allowance_s':transfers*.2,'tool_and_allowance_s':dwell,
            'mechanical_execution_s':mechanical,'whole_cycle_s':mechanical,
            'search_budget_s':THINK_BUDGET_S,'mechanical_plus_search_budget_s':mechanical+THINK_BUDGET_S,
            'source_to_destination_horizontal_s':carrying})
    result['cases'].append({**case,'route_xy_mm':route,**branch,'timings':timings})
result['scope']='机械执行从参考等待点通行高度静止出发，到抓取、搬运、放下、抬升、回等待点静止结束。吃子包括先移走目标棋子。纯运动、吸放、通信停稳及搜索预算分开。whole_cycle_s保留旧键，仍仅指机械执行，不是人机对战总响应时间；总响应还需视觉、引擎启动/合法性检查/退出等开销。速度加速度与工具延迟均为假设，非实测能力。'
result['collision_scope']='离线查4度CAD构型矩阵：关节路径每0.5度以内采样，要求相邻四个CAD格点均通过。没有新增SolidWorks连续碰撞验证；Z工作高度、工具和其他棋子未验收。不是可直接执行的运动计划。'
(OUT/'timing-scenarios.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
for c in result['cases']:
    print(c['name'],'branch',c['elbow_sign'],'deltas',np.array(c['delta_deg']).round(2).tolist())
    print([(t['profile'],round(t['whole_cycle_s'],2),round(t['horizontal_all_legs_s'],2),round(t['vertical_s'],2)) for t in c['timings']])
