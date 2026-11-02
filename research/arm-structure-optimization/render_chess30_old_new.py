"""同一棋盘同一摆位的新旧工作区对比，完全离线绘制。"""
import json, runpy, site
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Circle, Rectangle, Patch
from matplotlib.lines import Line2D
site.addsitedir('C:/Users/lst/AppData/Local/Temp/scara-workspace-python')
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import intersects_xy

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
shared=runpy.run_path(str(OUT/'render_equal275_chess30.py'))
board,center,rot=shared['board'],shared['center'],shared['rot']
local=np.array([(xx,yy) for yy in range(-180,181,40) for xx in range(-160,161,40)])
points=center+local@rot
fig,axes=plt.subplots(1,2,figsize=(17,11),facecolor='#f7fafb')
fig.text(.065,.95,'新旧机械臂 ＋ 同一棋盘摆位',fontsize=25,color='#17354a')
fig.text(.065,.909,'棋子 Ø30 mm · 交点间距 40 mm · 网格 320×360 mm · 中心向左 210 mm、向前 210 mm · 棋盘边斜置 45°',fontsize=12,color='#566e7f')
records=[]
for ax,(name,l1,path) in zip(axes,[
    ('旧版 195＋275 mm',195,ROOT/'output/constrained-workspace-2026-10-09/cad-collision-scan.npz'),
    ('新版 275＋275 mm',275,ROOT/'hardware/arm_equal_275/v1/analysis/workspace.npz')]):
    data=np.load(path);angles,mask=data['angles'],data['connected']
    a,b=np.meshgrid(np.deg2rad(angles),np.deg2rad(angles),indexing='ij')
    x=-l1*np.sin(a)-275*np.sin(a+b);y=l1*np.cos(a)+275*np.cos(a+b)
    cells=[];branch=[]
    for i in range(len(angles)-1):
        for j in range(len(angles)-1):
            if not np.all(mask[i:i+2,j:j+2]):continue
            p=[(x[i,j],y[i,j]),(x[i+1,j],y[i+1,j]),(x[i+1,j+1],y[i+1,j+1]),(x[i,j+1],y[i,j+1])]
            cells.append(p)
            if angles[j]>=0:branch.append(p)
    def field(polys):
        values=[Polygon(p) for p in polys]
        return unary_union([p if p.is_valid else p.buffer(0) for p in values])
    allfield,single=field(cells),field(branch)
    passed=intersects_xy(allfield,points[:,0],points[:,1])
    one=intersects_xy(single,points[:,0],points[:,1])
    missing=points[~passed];switch=points[passed & ~one]
    records.append({'version':name,'same_branch_passed':int(one.sum()),'any_branch_passed':int(passed.sum()),
                    'not_covered_local_mm':local[~passed].tolist(),'not_covered_robot_mm':missing.tolist()})
    ax.set_aspect('equal');ax.set_facecolor('white')
    ax.add_patch(Circle((0,0),l1+275,fc='#f0e7e3',ec='#b6a59d',ls='--',lw=1,zorder=1))
    ax.add_collection(PolyCollection(cells,facecolors='#c6e3df',edgecolors='none',antialiaseds=False,zorder=2))
    ax.add_collection(PolyCollection(branch,facecolors='#48a69a',edgecolors='none',antialiaseds=False,zorder=3))
    ax.axvline(0,color='#91a9b7',ls='--',lw=.6,zorder=4);ax.axhline(0,color='#91a9b7',ls='--',lw=.6,zorder=4)
    ax.plot([0,0,0],[0,l1,l1+275],'-o',lw=2.5,ms=5,color='#f4bd4a',zorder=5)
    ax.add_patch(Rectangle((-20,-174),40,40,fc='#596b7d',zorder=5))
    ax.annotate('后方立柱',xy=(0,-154),xytext=(105,-220),fontsize=10,arrowprops={'arrowstyle':'->'},zorder=9)
    ax.plot(0,0,'o',color='#17354a',ms=6,zorder=9)
    ax.annotate('肩轴 O',xy=(0,0),xytext=(100,35),fontsize=11,arrowprops={'arrowstyle':'->'},zorder=9)
    board(ax,lambda p:center+p@rot,6.7,7)
    ax.plot(*center,'+',ms=10,mew=1.5,color='#17354a',zorder=12)
    if len(missing):
        ax.scatter(missing[:,0],missing[:,1],s=135,facecolors='none',edgecolors='#d51542',lw=2,zorder=14)
    if len(switch):
        ax.scatter(switch[:,0],switch[:,1],s=100,marker='s',facecolors='none',edgecolors='#a76100',lw=1.5,zorder=14)
    ax.set_xlim(-610,610);ax.set_ylim(-610,610)
    ax.set_xticks(np.arange(-600,601,200));ax.set_yticks(np.arange(-600,601,200))
    ax.set_xlabel('横向 X / mm');ax.set_ylabel('前方 Y / mm')
    ax.set_title(name,fontsize=17,pad=17,color='#17354a')
    for spine in ax.spines.values():spine.set_color('#c4d1da')
    ax.text(-555,-415,f'同一肘型：{int(one.sum())} / 90 点通过初筛\n全部肘型合计：{int(passed.sum())} / 90 点通过初筛',
            fontsize=11,color='#17354a',linespacing=1.7,bbox={'fc':'white','ec':'none','pad':5},zorder=15)
    ax.text(-555,-535,'相同棋盘中心：(−210, 210) mm',fontsize=10,color='#17354a',zorder=15)
fig.subplots_adjust(left=.06,right=.97,bottom=.25,top=.84,wspace=.17)
fig.legend(handles=[Patch(fc='#48a69a',label='同一肘型的采样可达区'),Patch(fc='#c6e3df',label='其他肘型补充区'),
                    Patch(fc='#f0e7e3',label='碰撞剔除或采样未覆盖'),
                    Line2D([],[],marker='o',mfc='none',mec='#d51542',mew=2,ls='none',label='各肘型均未覆盖的交点')],
           loc='lower center',bbox_to_anchor=(.5,.175),ncol=4,frameon=False,fontsize=10)
if any(r['any_branch_passed']!=r['same_branch_passed'] for r in records):
    fig.text(.065,.16,'棕色方框：所选肘型未覆盖，但另一肘型有样本覆盖。',fontsize=10,color='#a76100')
fig.text(.065,.133,'对比沿用新版候选摆位；没有为旧版重新优化棋盘位置，不能据此断言旧版在其他摆位也无法覆盖。',fontsize=11,color='#17354a')
fig.text(.065,.095,'口径差异：旧版未含被抑制的传动总成；新版已含同步轮与皮带。两版均为当前 Z / R 姿态的 4° CAD 碰撞采样。',fontsize=10,color='#566e7f')
fig.text(.065,.057,'棋子仅作尺寸示意，未指定外板 / 边框尺寸；抓手、棋子实体及连续走棋路径尚未验证。',fontsize=10,color='#ad493f')
for ext in ['png','svg']:fig.savefig(OUT/f'chess30-old-new-top-view.{ext}',dpi=180)
plt.close(fig)
(OUT/'chess30-old-new-top-view.json').write_text(json.dumps({'layout':shared['layout'],'results':records,'scope':'采样单元插值初筛；同盘同摆位；不是连续碰撞验证。'},ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(records,ensure_ascii=False))
