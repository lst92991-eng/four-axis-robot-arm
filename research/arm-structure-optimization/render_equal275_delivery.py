"""生成等臂副本的工作区图与交付包；不再操作SolidWorks。"""
import json,math,site,zipfile,hashlib,runpy
from pathlib import Path
import numpy as np
site.addsitedir('C:/Users/lst/AppData/Local/Temp/scara-workspace-python')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Circle,Rectangle
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'hardware/arm_equal_275/v1';A=OUT/'analysis'
d=np.load(A/'workspace.npz');angles=d['angles'];mask=d['connected']
summary=json.loads((A/'summary.json').read_text('utf-8'))
model=json.loads((OUT/'final-model-verification.json').read_text('utf-8'))
assert not model['part_errors'] and not model['new_assembly_errors'] and not model['component_position_mismatches']
q1,q2=np.meshgrid(np.deg2rad(angles),np.deg2rad(angles),indexing='ij')
x=-275*np.sin(q1)-275*np.sin(q1+q2);y=275*np.cos(q1)+275*np.cos(q1+q2)
cells=[]
for i in range(len(angles)-1):
    for j in range(len(angles)-1):
        if np.all(mask[i:i+2,j:j+2]):cells.append([(x[i,j],y[i,j]),(x[i+1,j],y[i+1,j]),(x[i+1,j+1],y[i+1,j+1]),(x[i,j+1],y[i,j+1])])
r=np.hypot(x[mask],y[mask]);rmin=float(r.min());rmax=float(r.max())
plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'font.size':11,'svg.fonttype':'path','figure.facecolor':'#f7fafb'})
fig,(ax,z)=plt.subplots(1,2,figsize=(15,9),gridspec_kw={'width_ratios':[1.18,1]})
fig.suptitle('275＋275 mm 等臂版 · 机械臂工作区俯视图',x=.065,ha='left',y=.968,fontsize=23,color='#17354a')
fig.text(.065,.916,'来自独立装配副本的207个实体 · 8,281组关节构型 · 4°采样 · 同步轮与皮带外形已纳入',fontsize=11,color='#566e7f')
for a in (ax,z):
    a.set_aspect('equal');a.set_facecolor('white');a.grid(color='#dce5eb',lw=.6,zorder=0)
    a.add_patch(Circle((0,0),550,facecolor='#f0e7e3',edgecolor='#c1b1aa',ls='--',lw=1.2,zorder=1))
    a.add_collection(PolyCollection(cells,facecolors='#42a397',edgecolors='none',antialiaseds=False,zorder=2))
    a.plot(0,0,'o',color='#17354a',ms=6,zorder=6);a.axvline(0,color='#a1b2bd',lw=.6,ls='--',zorder=3)
    a.set_xlabel('横向 X / mm');a.set_ylabel('前方 Y / mm')
    for sp in a.spines.values():sp.set_color('#bdcbd4')
ax.set_xlim(-610,610);ax.set_ylim(-610,610)
ax.set_title('全工作区：绿色为采样支持的无碰撞位置',fontsize=13,pad=14)
ax.plot([0,0,0],[0,275,550],'-o',color='#f4bd4a',lw=3,ms=6,zorder=5)
ax.text(20,137.5,'G1：275 mm',color='#17354a',fontsize=10,zorder=6)
ax.text(20,412.5,'G2：275 mm',color='#17354a',fontsize=10,zorder=6)
ax.annotate('当前伸直姿态的转轴连线',xy=(0,550),xytext=(-510,585),fontsize=10,arrowprops={'arrowstyle':'->','color':'#17354a'},zorder=7)
ax.text(-560,-550,'绿色：腕轴中心可达位置\n灰褐色：碰撞约束剔除或采样未覆盖区域\n虚线：550 mm 理论最远边界',fontsize=10,color='#17354a',bbox={'facecolor':'white','edgecolor':'#d6e1e7','pad':7},zorder=7)
z.set_xlim(-220,220);z.set_ylim(-260,230)
z.set_title('中心放大：等长后仍有结构碰撞盲区',fontsize=13,pad=14)
z.text(10,8,'肩轴 O',color='#17354a',fontsize=11,zorder=7)
z.add_patch(Rectangle((-20,-174),40,40,color='#596b7d',zorder=5))
z.annotate('后方立柱投影',xy=(0,-154),xytext=(60,-215),fontsize=10,arrowprops={'arrowstyle':'->','color':'#17354a'},zorder=7)
z.text(0,190,f'已通过样本的最小半径约 {rmin:.1f} mm',ha='center',fontsize=10,color='#17354a',bbox={'facecolor':'white','edgecolor':'none','pad':4},zorder=7)
fig.text(.065,.08,'范围口径：当前装配Z高度、末端R相对姿态不变；绿色表示腕轴中心位置，不是全部实体的扫掠外包络。',fontsize=10)
fig.text(.065,.045,'4°离散边界不能直接用作实机软限位；未加入吸盘、软管、动态挠度与实物误差。最小半径数值是采样结果。',fontsize=10,color='#b94d43')
fig.subplots_adjust(left=.065,right=.97,bottom=.16,top=.85,wspace=.24)
for ext in ('png','svg'):fig.savefig(OUT/f'workspace-top-view.{ext}',dpi=180)
plt.close(fig)
metrics={'minimum_sampled_radius_mm':rmin,'maximum_sampled_radius_mm':rmax,'filled_configuration_cells':len(cells),
    'scope':summary['scope'],'new_assembly_errors':model['new_assembly_errors'],'source_inherited_mate_errors':len([e for e in model['inherited_assembly_errors'] if not e['warning']])}
(A/'plot-metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2),encoding='utf-8')
runpy.run_path(str(Path(__file__).with_name('render_equal275_comparison.py')))
readme=f'''275+275 mm 等臂版 CAD 副本（2026-10-09）

打开 native/EQ275_SCARA四轴机械臂总成.SLDASM。必须保留整个native文件夹及同目录零件，不要只移动装配文件。
也可使用独立中性格式 SCARA_equal_275.step。

本次修改
- G1转轴中心距：195 -> 275 mm（增加80 mm）；G2仍为275 mm。
- G1臂板外形：330 × 60 × 10 mm；原孔径、板厚和端部安装尺寸保留。
- 中槽直段50 -> 130 mm，并修正槽位置、电机槽组和肘部四孔，避免中心矩形变长导致它们漂移。
- 肘部组件及大同步轮前移80 mm；关闭原CAD皮带的固定长度驱动，让皮带按新中心距重建。
- 原生装配、零件和虚拟皮带依赖均为独立副本。本次恢复仅打开和保存EQ275文件。

验证
- SolidWorks圆柱面回读轴距：{model['links_mm'][0]:.6f} / {model['links_mm'][1]:.6f} mm。
- G1体积：{model['G1_volume_mm3']:.6f} mm³，与保留两端、中间加长80mm的几何结果一致。
- G1特征无错误；没有新增装配配合错误；原模型既有{metrics['source_inherited_mate_errors']}项配合错误和配合组警告保留，见final-model-verification.json。
- 207实体，8281组4°构型；{summary['connected']}组无碰撞样本属于直臂姿态所在离散连通分量。
- 五对既有轴/轴套/键连接交叠按原模型解释排除；同步轮、皮带当前外形纳入本次检查。
- 新图为当前Z高度和末端R相对姿态的几何分析；不是连续路径证明、强度或实机安全验收。

需要后续核实
- 实物同步带长度/齿数必须重新选型，不能从CAD带长直接推出采购规格。
- 加长后的刚度、驱动负担、线缆/软管、抓手TCP和实物限位尚未验证。
- 当前棋盘尚未确定，本图没有把任何棋盘尺寸认定为最终实购物料。
- 此尺寸版本保留原生草图；以后再改臂长，仍须同步复核孔槽位置及装配关系。

文件
workspace-comparison.png / .svg：旧版195+275与新版275+275的同尺度俯视对比及中心放大。
旧版采样未纳入被抑制的同步轮/皮带总成，新版已纳入；不能将面积差解释为纯臂长修改的精确增益。
workspace-top-view.png / .svg：新的工作区俯视图。
model-isometric.png：新模型预览。
analysis/：实体干涉采样、连接交叠排除和统计证据。
final-model-verification.json：最终装配核验；早期制作日志中的错误已由后续步骤修正。

原件记录
初次复制时原文件哈希未变。用户关闭/重启SolidWorks的间隔内，原总装及其旧临时虚拟总装的哈希发生变化；该差异保留在日志中，没有回滚。恢复后的操作只针对EQ275副本，原件相对恢复基线未再变化。
'''
(OUT/'README.txt').write_text(readme,encoding='utf-8')
status={'status':'complete','native_assembly':'native/EQ275_SCARA四轴机械臂总成.SLDASM','step':'SCARA_equal_275.step','workspace_plot':'workspace-comparison.png','new_version_detail_plot':'workspace-top-view.png','links_mm':[275,275],
    'validation':metrics,'remaining':[],'hardware_validation_complete':False}
(OUT/'STATUS.json').write_text(json.dumps(status,ensure_ascii=False,indent=2),encoding='utf-8')
stale=OUT/'尚未完成_请勿作为交付模型使用.txt'
if stale.exists():stale.unlink()
chosen=[p for p in (OUT/'native').rglob('*') if p.is_file() and not p.name.startswith('~$')]
chosen +=[OUT/name for name in ['SCARA_equal_275.step','workspace-comparison.png','workspace-comparison.svg','workspace-top-view.png','workspace-top-view.svg','model-isometric.png','README.txt','STATUS.json','final-model-verification.json','part-repair-check.json','belt-relocation.json']]
chosen +=[p for p in A.glob('*') if p.is_file() and p.name!='checkpoint.npz']
manifest={str(p.relative_to(OUT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in chosen}
(OUT/'DELIVERY-SHA256.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8');chosen.append(OUT/'DELIVERY-SHA256.json')
archive=OUT/'SCARA_equal275_v1.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as zf:
    for p in chosen:zf.write(p,str(p.relative_to(OUT)))
with zipfile.ZipFile(archive) as zf:assert zf.testzip() is None
print(json.dumps({'metrics':metrics,'archive':str(archive),'files':len(chosen),'zip_bytes':archive.stat().st_size},ensure_ascii=False))
