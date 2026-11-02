"""新版碰撞采样工作区叠加30mm象棋、40mm交点间距的候选摆位。"""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Circle, Polygon, Rectangle, Patch

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'research/arm-structure-optimization'
info=json.loads((OUT/'equal275-chess30-layout.json').read_text('utf-8'))
layout=next(v for v in info['results'] if v['pitch_mm']==40)['best_sampled_layout']
center=np.array(layout['center_mm'])
theta=np.deg2rad(layout['rotation_ccw_deg'])
rot=np.array([[np.cos(theta),np.sin(theta)],[-np.sin(theta),np.cos(theta)]])
d=np.load(ROOT/'hardware/arm_equal_275/v1/analysis/workspace.npz')
angles,mask=d['angles'],d['connected']
a,b=np.meshgrid(np.deg2rad(angles),np.deg2rad(angles),indexing='ij')
x=-275*np.sin(a)-275*np.sin(a+b);y=275*np.cos(a)+275*np.cos(a+b)
all_cells=[];branch_cells=[]
for i in range(len(angles)-1):
    for j in range(len(angles)-1):
        if not np.all(mask[i:i+2,j:j+2]):continue
        p=[(x[i,j],y[i,j]),(x[i+1,j],y[i+1,j]),(x[i+1,j+1],y[i+1,j+1]),(x[i,j+1],y[i,j+1])]
        all_cells.append(p)
        if angles[j]>=0:branch_cells.append(p)

plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'svg.fonttype':'path','font.size':11})
fig=plt.figure(figsize=(16,10),facecolor='#f7fafb')
ax=fig.add_axes([.065,.20,.53,.63]);detail=fig.add_axes([.65,.27,.29,.54])
fig.text(.065,.946,'新版机械臂 ＋ 中国象棋棋盘',fontsize=25,color='#17354a')
fig.text(.065,.904,'G1 275 mm / G2 275 mm   ·   棋子直径 30 mm   ·   横纵交点间距 40 mm',fontsize=12,color='#566e7f')
ax.set_aspect('equal');ax.set_facecolor('white')
ax.add_patch(Circle((0,0),550,fc='#f0e7e3',ec='#b6a59d',ls='--',lw=1,zorder=1))
ax.add_collection(PolyCollection(all_cells,facecolors='#c6e3df',edgecolors='none',antialiaseds=False,zorder=2))
ax.add_collection(PolyCollection(branch_cells,facecolors='#48a69a',edgecolors='none',antialiaseds=False,zorder=3))
ax.axvline(0,color='#8dabb7',lw=.7,ls='--',zorder=4)
ax.axhline(0,color='#8dabb7',lw=.7,ls='--',zorder=4)
ax.plot([0,0,0],[0,275,550],'-o',lw=2.5,ms=5,color='#f4bd4a',zorder=5)
ax.text(20,320,'伸直姿态\n转轴连线',fontsize=10,color='#715311',zorder=8)
ax.add_patch(Rectangle((-20,-174),40,40,fc='#596b7d',zorder=5))
ax.annotate('后方立柱',xy=(0,-154),xytext=(100,-215),fontsize=10,arrowprops={'arrowstyle':'->'},zorder=9)
ax.plot(0,0,'o',color='#17354a',ms=7,zorder=9)
ax.annotate('肩轴原点 O',xy=(0,0),xytext=(100,35),fontsize=11,color='#17354a',arrowprops={'arrowstyle':'->'},zorder=9)
ax.plot([0,center[0]],[0,center[1]],ls='--',color='#263e54',lw=1,zorder=7)
ax.set_xlim(-610,610);ax.set_ylim(-610,610)
ax.set_xticks(np.arange(-600,601,200));ax.set_yticks(np.arange(-600,601,200))
ax.set_xlabel('横向 X / mm（左侧为负）');ax.set_ylabel('前方 Y / mm')
ax.set_title('候选摆位：左前方，棋盘边斜置 45°',fontsize=14,pad=14)
for spine in ax.spines.values():spine.set_color('#c4d1da')

def board(axis, transform, letter_size, river_size):
    def line(points,**kwargs):
        p=transform(np.array(points));axis.plot(p[:,0],p[:,1],**kwargs)
    corners=np.array([[-160,-180],[160,-180],[160,180],[-160,180]])
    axis.add_patch(Polygon(transform(corners),fc='#fff6dc',ec='#6e6147',lw=1.2,zorder=6))
    for yy in range(-180,181,40):line([[-160,yy],[160,yy]],color='#8b7a55',lw=.75,zorder=7)
    for xx in range(-160,161,40):
        segments=[(-180,180)] if abs(xx)==160 else [(-180,-20),(20,180)]
        for low,high in segments:line([[xx,low],[xx,high]],color='#8b7a55',lw=.75,zorder=7)
    for lo,hi in [(-180,-100),(100,180)]:
        line([[-40,lo],[40,hi]],color='#8b7a55',lw=.75,zorder=7)
        line([[40,lo],[-40,hi]],color='#8b7a55',lw=.75,zorder=7)
    pts=transform(np.array([(xx,yy) for yy in range(-180,181,40) for xx in range(-160,161,40)]))
    axis.scatter(pts[:,0],pts[:,1],s=4,color='#8b7a55',zorder=8)
    for xx,text in [(-80,'楚河'),(80,'汉界')]:
        point=transform(np.array([[xx,0]]))[0]
        axis.text(*point,text,fontsize=river_size,ha='center',va='center',color='#8b7a55',zorder=8)
    pieces=[]
    for row,chars in [(0,'车马相仕帅仕相马车'),(9,'車馬象士將士象馬車')]:
        pieces.extend((col,row,char) for col,char in enumerate(chars))
    for row in [2,7]:pieces.extend((col,row,'炮' if row==2 else '砲') for col in [1,7])
    for row in [3,6]:pieces.extend((col,row,'兵' if row==3 else '卒') for col in range(0,9,2))
    assert len(pieces)==32
    for col,row,char in pieces:
        point=transform(np.array([[col*40-160,row*40-180]]))[0]
        color='#b63732' if row<5 else '#243c4b'
        axis.add_patch(Circle(point,15,fc='#fffcf1',ec=color,lw=1,zorder=10))
        axis.text(*point,char,ha='center',va='center',fontsize=letter_size,color=color,zorder=11)

board(ax,lambda p:center+p@rot,7,8)
ax.plot(*center,'+',ms=10,mew=1.5,color='#17354a',zorder=12)
ax.annotate('棋盘中心 (−210, 210) mm',xy=center,xytext=(-555,-395),
            fontsize=11,color='#17354a',arrowprops={'arrowstyle':'->','connectionstyle':'angle3,angleA=0,angleB=90'},
            bbox={'fc':'white','ec':'none','pad':4},zorder=12)
ax.text(-555,-510,'向左 210 mm，向前 210 mm\n90 / 90 个交点通过采样工作区初筛',fontsize=11,color='#17354a',zorder=12)

detail.set_aspect('equal');board(detail,lambda p:p,12,14)
detail.set_xlim(-205,210);detail.set_ylim(-220,230);detail.axis('off')
detail.set_title('棋盘细节（转正显示）',fontsize=14,pad=18)
detail.annotate('',xy=(-160,-210),xytext=(160,-210),arrowprops={'arrowstyle':'<->','color':'#17354a'})
detail.text(0,-219,'320 mm · 9 列交点',ha='center',va='top',fontsize=11,color='#17354a')
detail.annotate('',xy=(193,-180),xytext=(193,180),arrowprops={'arrowstyle':'<->','color':'#17354a'})
detail.text(200,0,'360 mm · 10 行交点',rotation=90,ha='left',va='center',fontsize=11,color='#17354a')
fig.text(.665,.231,'图中圆片直径 30 mm，展示 32 枚开局棋子。\n相邻交点 40 mm，棋子之间留 10 mm 空隙。',fontsize=11,color='#17354a',linespacing=1.7)
fig.legend(handles=[Patch(fc='#48a69a',label='所选单肘型的采样可达区'),Patch(fc='#c6e3df',label='其他肘型补充区域'),Patch(fc='#f0e7e3',label='碰撞剔除或采样未覆盖')],
           loc='lower center',bbox_to_anchor=(.5,.111),ncol=3,frameon=False,fontsize=11)
fig.text(.065,.089,'尺寸指最外侧落子交点跨度；图中未指定棋盘外板、边框及定位标记尺寸。',fontsize=11,color='#566e7f')
fig.text(.065,.052,'基于当前 Z / R 姿态的 4° CAD 碰撞采样。棋子仅作尺寸与摆位示意，抓手、棋子及连续抬升路径尚未做碰撞验证。',fontsize=10,color='#ad493f')
for ext in ['png','svg']:fig.savefig(OUT/f'equal275-chess30-top-view.{ext}',dpi=180)
plt.close(fig)
print(OUT/'equal275-chess30-top-view.png')
