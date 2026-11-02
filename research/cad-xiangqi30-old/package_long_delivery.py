"""离线整理分析结果和交付清单；不操作CAD。"""
import json,hashlib,zipfile,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'hardware/xiangqi30-long275275-2026-10-13'
A=ROOT/'output/workspace-comparison-2026-10-13'
m=json.loads((A/'plot-metrics.json').read_text('utf-8'))
summary=json.loads((A/'comparison-summary.json').read_text('utf-8'))
table=['| 臂长 / mm | 两颗螺丝头高 / mm | 采样投影面积 / cm² | 棋盘端点通过 |','|---|---:|---:|---:|']
for key in ('old_head6','old_head3p82','new_head6','new_head3p82'):
    r=m[key];table.append(f'| {r["links_mm"][0]}＋275 | {r["head_height_mm"]} | {r["sampled_field_area_cm2"]:.0f} | {r["board_free_endpoints"]}/90 |')
boundaries=[]
for key in ('old_head6','old_head3p82','new_head6','new_head3p82'):
    r=m[key];parts=[]
    for b in r['first_axis_contact_boundary']:
        parts.append(f'{b["free_deg"]:.3f}°（下一干涉：'+ ' / '.join(b['collision'])+'）')
    boundaries.append(f'- {r["links_mm"][0]}＋275、头高{r["head_height_mm"]} mm：'+ '；'.join(parts))
report='''# 新旧机械臂工作区俯视对比（2026-10-13）

## 怎么看图

- `old-new-top-view.png`：195＋275与275＋275，均为6 mm原头，与两份原生总装一致；同坐标、同比例、同一棋盘摆位。
- `old-new-low-heads-top-view.png`：两版均按两颗3.82 mm低头螺丝的分析变体对比。
- `screw-change-top-view.png`：195＋275保持不变，比较6 mm原头与3.82 mm低头。
- `all-four-top-view.png`：两种臂长×两种头高，方便分开判断两项改动。
- 绿色/蓝色表示构型采样支持的腕轴中心位置；淡褐色表示理论半径内未获支持区域；红叉是精确端点干涉不通过的棋盘交点。

## 本次结果

'''+ '\n'.join(table)+f'''

旧版换低头后，采样投影净增约{m['lower_heads_old']['net_area_change_cm2']:.0f} cm²；旧版/新版同用低头时，新版净增约{m['long_arm_low_heads']['net_area_change_cm2']:.0f} cm²。这是离散构型单元经FK投影、合并后的近似面积，不能作为连续可达率或实机性能指标。

## 螺丝判断

只改`M6内六角x22-1`和`M6内六角x22-5`两颗。原承压面保持不变，头部从6截低至3.82 mm，杆部不平移；已用实体极值验证头部切面和杆底。原CAD头部轮廓截低是本次几何假设，实际商品的头径、垫片和安装误差还需核对。

现有G1底面与新头部的名义竖直间隙约2.08 mm；这是静态CAD间隙，不包含挠曲、装配误差和振动。

从直臂开始，第一臂相对基座首次接触的精细角度区间：

'''+ '\n'.join(boundaries)+'''

这些角度只说明第一臂—基座首次干涉，不能单独当作整机安全关节软限位。图的全部构型还检验了两臂之间及第二臂与基座之间的干涉。

## 公平比较范围

- 每版统一153个顶层零件、204个实体，实体BRep干涉检查，均排除传动子总成；没有混用历史“旧版不含传动、新版含传动”的面积口径。
- 排除5对已记录的轴/轴套/键连接交叠；其他初始跨组干涉会让扫描停止。
- 当前装配Z高度、腕相对姿态固定；肩和肘按4°网格，共8,281组构型。使用与直臂相连的离散分量；它并不证明相邻构型之间的连续运动无碰撞。
- 棋盘90点另行解两支IK并做CAD端点干涉检查；表中的通过数仍不是从任意起点搬运到该点的路径证明。
- 本次板315×350×14 mm、格距35 mm、棋子直径30 mm，是支架设计参考；中心在肩轴前方(0,280) mm。90点网格跨度280×315 mm；含四码的外包络450×485 mm。
- 尚未纳入传动总成、完整电磁铁、线缆、实际TCP、抓放高度、动态挠度与真实装配误差。电磁铁尚需实物模型；不能把腕轴坐标直接当磁铁接触点。
- 绿区是末端腕轴中心位置，不是机械臂实体扫掠外轮廓；也不是全部Z高度的三维工作空间。

## 可复现证据

四个NPZ保存角度、无碰撞网格及连通网格；对应JSON保存模型、实体范围、接触排除、棋盘90点及首接触角度。`comparison-summary.json`汇总；`plot-metrics.json`保存投影面积。扫描与绘图脚本在主项目`research/cad-xiangqi30-old/`，只在临时实体上截低螺丝，没有修改或保存源CAD。

API依据：[SolidWorks CreateBodyFromBox3](https://help.solidworks.com/2016/english/api/sldworksapi/SOLIDWORKS.Interop.sldworks~SOLIDWORKS.Interop.sldworks.IModeler~CreateBodyFromBox3.html)。临时裁切盒按底面中心、轴向、宽、长、高构建，并通过实体极值回读确认。
'''
(A/'对比说明.md').write_text(report,'utf-8')
shutil.copytree(A,OUT/'workspace-comparison',dirs_exist_ok=True)
readme='''# 275＋275 mm机械臂＋30 mm象棋参考总装

解压整个文件夹后，用SolidWorks 2026打开：

`SolidWorks/SCARA_FRONT_275_275_CHESS30.SLDASM`

不要只复制SLDASM。中性格式为`SCARA_FRONT_275_275_CHESS30.step`。

## 内容

- 06线程交付的275＋275 mm候选机械结构，棋盘摆位与已交付的旧机版一致。
- 32枚参考棋子，红黑14种身份，直径30 mm、高8 mm；字面为自建参考实体，不是已采购商品外形。
- 棋盘315×350×14 mm、格距35 mm；黑方近机械臂、红方朝操作者。
- 四个独立ArUco ID0～3，DICT_4X4_50；黑码48 mm、白边各6 mm，外形60 mm。打印稿在`markers/`，以100%原尺寸打印。
- 四码总外包络450×485 mm；用户提供的RV1106 PCB STEP在`reference/`。
- 实际CAD预览在`preview/`，新旧工作区对比及完整证据在`workspace-comparison/`。

## 验证与边界

- 总装独立重开、38个顶层组件；原生引用均在包内，虚拟传动总装与皮带整理为独立文件。
- 圆柱面轴线回读确认275＋275 mm；棋具和标记的37个变换与旧机最终版相同。原来的配合问题保留，无新增机械臂特征错误。
- **本总装仍保留原CAD的6 mm螺丝头。** 3.82 mm低头是工作区分析变体，没有擅自改成某款实购物料；对比图明确注明头高。
- 新版仍是候选结构，未当成当前实装机械臂；棋盘、磁吸能力、摄像头高度与支架尺寸尚未冻结。现有总装用于设计参考。
- 工作区是当前Z/R的离散几何分析，不是运动路径、完整电磁铁/布线或实机验收。
- 原新臂47个和旧总装71个原生文件均与各自既有归档SHA-256一致；本次只操作交付副本。
- 按用户已授权处理参考副本的CAM保存提示，不保留副本CAM加工数据。没有实物动作、commit或push。

文件完整性清单为`SHA256SUMS.json`；原生/摆位验证为`validation.json`及`assembly-layout.json`。
'''
(OUT/'README_先读我.md').write_text(readme,'utf-8')
files=[p for p in OUT.rglob('*') if p.is_file() and not p.name.startswith('~$') and p.suffix.lower()!='.bmp' and p.name!='SHA256SUMS.json']
assert all(p.stat().st_size>0 for p in files)
manifest={p.relative_to(OUT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
(OUT/'SHA256SUMS.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),'utf-8');files.append(OUT/'SHA256SUMS.json')
archive=ROOT/'output/SCARA_front_275-275_30mm_chess_2026-10-13.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for p in files:z.write(p,'SCARA_front_275275_30mm/'+p.relative_to(OUT).as_posix())
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    for name,h in manifest.items():assert hashlib.sha256(z.read('SCARA_front_275275_30mm/'+name)).hexdigest()==h
print(json.dumps({'zip':str(archive),'bytes':archive.stat().st_size,'files':len(files),'sha256':hashlib.sha256(archive.read_bytes()).hexdigest()},ensure_ascii=False))
