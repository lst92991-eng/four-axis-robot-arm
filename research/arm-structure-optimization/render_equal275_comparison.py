"""从两版已保存的CAD碰撞样本绘制同尺度对比，不连接SolidWorks。"""
import hashlib
import json
import shutil
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Circle, Rectangle, Patch

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'hardware/arm_equal_275/v1'
OLD = ROOT / 'output/constrained-workspace-2026-10-09'
plt.rcParams.update({'font.family': 'Microsoft YaHei', 'axes.unicode_minus': False,
                     'font.size': 11, 'svg.fonttype': 'path'})
fig, axes = plt.subplots(2, 2, figsize=(15, 13), facecolor='#f7fafb')
fig.suptitle('机械臂工作区俯视对比', x=.065, y=.978, ha='left', fontsize=25, color='#17354a')
fig.text(.065, .941, '同一肩轴原点、前方方向与坐标比例 · 上排全工作区，下排中心放大 · 单位 mm', color='#566e7f')
metrics = []
for col, (name, l1, path, detail) in enumerate([
    ('旧版  G1 195 ＋ G2 275 mm', 195, OLD/'cad-collision-scan.npz', '原扫描：未纳入被抑制的同步轮 / 皮带总成'),
    ('新版  G1 275 ＋ G2 275 mm', 275, OUT/'analysis/workspace.npz', '新扫描：已纳入修正后的同步轮 / 皮带外形'),
]):
    data = np.load(path)
    angles, mask = data['angles'], data['connected']
    a, b = np.meshgrid(np.deg2rad(angles), np.deg2rad(angles), indexing='ij')
    x, y = -l1*np.sin(a)-275*np.sin(a+b), l1*np.cos(a)+275*np.cos(a+b)
    cells = []
    for i in range(len(angles)-1):
        for j in range(len(angles)-1):
            if np.all(mask[i:i+2,j:j+2]):
                cells.append([(x[i,j],y[i,j]),(x[i+1,j],y[i+1,j]),
                              (x[i+1,j+1],y[i+1,j+1]),(x[i,j+1],y[i,j+1])])
    radii = np.hypot(x[mask], y[mask])
    metrics.append({'version':name, 'links_mm':[l1,275], 'sample_count':int(mask.sum()),
                    'minimum_sampled_radius_mm':float(radii.min()),
                    'maximum_sampled_radius_mm':float(radii.max()),
                    'source':str(path.relative_to(ROOT)),
                    'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(), 'scope_difference':detail})
    for row in range(2):
        ax = axes[row, col]
        ax.set_aspect('equal'); ax.set_facecolor('white')
        ax.add_patch(Circle((0,0),l1+275,facecolor='#f0e7e3',edgecolor='#b7a59d',ls='--',lw=1.2,zorder=1))
        ax.add_collection(PolyCollection(cells,facecolors='#42a397',edgecolors='none',antialiaseds=False,zorder=2))
        ax.axvline(0,color='#a7b5be',lw=.6,ls='--',zorder=3)
        ax.axhline(0,color='#a7b5be',lw=.6,ls='--',zorder=3)
        ax.plot(0,0,'o',color='#17354a',ms=5,zorder=6)
        ax.set_xlabel('横向 X / mm'); ax.set_ylabel('前方 Y / mm')
        for spine in ax.spines.values(): spine.set_color('#c4d1da')
        if row == 0:
            ax.set_xlim(-610,610); ax.set_ylim(-610,610)
            ax.set_xticks(np.arange(-600,601,200)); ax.set_yticks(np.arange(-600,601,200))
            ax.set_title(name, fontsize=15, pad=12, color='#17354a')
            ax.plot([0,0,0],[0,l1,l1+275],'-o',color='#f4bd4a',lw=2.8,ms=5,zorder=5)
            ax.text(24,l1/2,f'G1 {l1}',fontsize=10,zorder=6)
            ax.text(24,l1+137.5,'G2 275',fontsize=10,zorder=6)
            ax.text(-565,-560,f'理论最远半径 {l1+275} mm',fontsize=10,
                    bbox={'facecolor':'white','edgecolor':'none','pad':4},zorder=7)
        else:
            ax.set_xlim(-250,250); ax.set_ylim(-250,250)
            ax.set_xticks(np.arange(-200,201,100)); ax.set_yticks(np.arange(-200,201,100))
            ax.set_title(f'中心放大 · 样本最近半径 {radii.min():.1f} mm',fontsize=13,pad=12)
            ax.add_patch(Rectangle((-20,-174),40,40,facecolor='#596b7d',zorder=5))
            ax.annotate('后方立柱',xy=(0,-154),xytext=(65,-210),fontsize=10,
                        arrowprops={'arrowstyle':'->','color':'#17354a'},zorder=7)
            ax.text(10,10,'肩轴 O',fontsize=10,zorder=7)
            ax.text(.5,-.2,detail,ha='center',transform=ax.transAxes,fontsize=10,color='#566e7f')
fig.legend(handles=[Patch(facecolor='#42a397',label='采样支持的无碰撞腕轴中心位置'),
                    Patch(facecolor='#f0e7e3',label='未通过碰撞检查或采样未覆盖'),
                    Patch(facecolor='#f4bd4a',label='伸直姿态转轴连线')],
           loc='lower center',bbox_to_anchor=(.5,.107),ncol=3,frameon=False,fontsize=10)
fig.text(.065,.084,'两版均采用 4° 关节采样、当前CAD高度与固定末端R相对姿态；绿色填色由四角均通过的采样单元绘出。',fontsize=10)
fig.text(.065,.057,'等臂长并未消除中心盲区。传动部件纳入范围不同，因此不把两图面积之差当作纯臂长改动的精确增益。',fontsize=10,color='#ad493f')
fig.text(.065,.03,'此图不证明连续路径可行；未加入抓手、软管、动态挠度与实物误差，不能直接作为实机限位。',fontsize=10,color='#566e7f')
fig.subplots_adjust(left=.075,right=.965,top=.89,bottom=.215,wspace=.17,hspace=.36)
for ext in ('png','svg'):
    fig.savefig(OUT/f'workspace-comparison.{ext}',dpi=180)
plt.close(fig)
shutil.copy2(OLD/'cad-collision-scan.npz', OUT/'analysis/old-workspace.npz')
shutil.copy2(OLD/'scan-summary.json', OUT/'analysis/old-scan-summary.json')
(OUT/'analysis/comparison-metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(metrics,ensure_ascii=False))
