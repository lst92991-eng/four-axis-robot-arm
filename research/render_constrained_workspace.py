"""把SolidWorks临时实体干涉结果映射成俯视工作区，不把未知区涂成可达。"""
import json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Polygon
from matplotlib.collections import PolyCollection

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output/constrained-workspace-2026-10-09'
data=np.load(OUT/'cad-collision-scan.npz')
angles=data['angles'];good=data['connected'];raw=data['good']
summary=json.loads((OUT/'scan-summary.json').read_text('utf-8'))
probe=json.loads((OUT/'boundary-and-board-probes.json').read_text('utf-8'))
zproof=json.loads((OUT/'z-projection-conclusion.json').read_text('utf-8')) if (OUT/'z-projection-conclusion.json').exists() else None
plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'font.size':10,
    'svg.fonttype':'path','figure.facecolor':'#f8fafc'})
q1,q2=np.meshgrid(np.deg2rad(angles),np.deg2rad(angles),indexing='ij')
x=-195*np.sin(q1)-275*np.sin(q1+q2);y=195*np.cos(q1)+275*np.cos(q1+q2)
cells=[]
for i in range(len(angles)-1):
    for j in range(len(angles)-1):
        if np.all(good[i:i+2,j:j+2]):cells.append([(x[i,j],y[i,j]),(x[i+1,j],y[i+1,j]),(x[i+1,j+1],y[i+1,j+1]),(x[i,j+1],y[i,j+1])])
fig,(a,b)=plt.subplots(1,2,figsize=(15,9),gridspec_kw={'width_ratios':[1.2,1]})
fig.suptitle('SCARA：加入机械结构碰撞后的 CAD 工作区',fontsize=22,x=.06,ha='left',y=.96,color='#183049')
fig.text(.06,.908,f'SolidWorks 实体干涉计算 · 腕轴中心 XY 投影 · {summary["angular_step_deg"]}°构型采样 · 原装配未修改',color='#56687a',fontsize=11)
for ax in (a,b):
    ax.set_aspect('equal');ax.set_facecolor('white');ax.grid(color='#dce4eb',lw=.5,zorder=0)
    for sp in ax.spines.values():sp.set_color('#cbd5df')
a.set_xlim(-510,510);a.set_ylim(-510,510)
a.add_patch(Circle((0,0),470,color='#eee7e4',zorder=1))
a.add_patch(Circle((0,0),470,fill=False,color='#a9a09c',ls='--',lw=1.3,zorder=2))
a.add_patch(Circle((0,0),80,color='white',zorder=2))
a.add_collection(PolyCollection(cells,facecolors='#50aaa0',edgecolors='none',antialiaseds=False,zorder=3))
a.plot(0,0,'ko',ms=5,zorder=7);a.text(10,4,'肩轴 O',fontsize=10,zorder=8)
old=json.loads((ROOT/'output/workspace-projection-2026-10-09/cad-projection-evidence.json').read_text('utf-8'))
static=['L型支架底座-1','L型支架竖板-1','40x40x500-1','铝型材挤压角座-1','铝型材挤压角座-2']
for c in old['components']:
    if c['name'] not in static:continue
    s=c['projection'];ps=[s['coordinates']] if s['type']=='Polygon' else s['coordinates'] if s['type']=='MultiPolygon' else []
    for p in ps:a.add_patch(Polygon(p[0],facecolor='#526379',edgecolor='white',lw=.5,zorder=5))
a.set_title('俯视：绿色为采样支持的无碰撞区域',fontsize=12,pad=12)
a.set_xlabel('横向 X / mm');a.set_ylabel('前方 Y / mm')
a.text(-460,-455,'灰褐色：理想圆环内，被结构约束剔除或未覆盖的区域\n绿色边界为离散近似，不能直接作为电机软限位',fontsize=9,color='#293d52',bbox={'facecolor':'white','edgecolor':'none','pad':5},zorder=9)
half=summary['angular_step_deg']/2
im=b.imshow(good.T,origin='lower',extent=[angles[0]-half,angles[-1]+half,angles[0]-half,angles[-1]+half],cmap=matplotlib.colors.ListedColormap(['#efe1dc','#50aaa0']),interpolation='nearest',aspect='equal')
b.plot(0,0,'ko',ms=5);b.set_xlabel('J1 相对当前直臂方向 / °');b.set_ylabel('J2 相对第一臂 / °')
b.set_title('构型空间：碰撞边界与两个关节有关',fontsize=12,pad=12)
b.set_xticks(range(-180,181,60));b.set_yticks(range(-180,181,60))
fig.text(.06,.075,'升降复核：模型327.5 mm行程未新增可达构型格点的XY投影；不代表同一XY点在每个Z高度都能到达。' if zproof else '已纳入：裸 CAD 的臂板、电机、支架、轴承座和紧固件；从直臂姿态筛选离散连通构型。',fontsize=10)
fig.text(.06,.043,'末端R保持当前相对姿态。未认证连续路径；未纳入抑制总成、实物额外挡块、吸盘及软管。不是实机安全限位。',color='#b7443d',fontsize=10)
fig.subplots_adjust(left=.06,right=.965,top=.87,bottom=.15,wspace=.23)
for ext in ('png','svg'):fig.savefig(OUT/f'cad-constrained-workspace.{ext}',dpi=180)
plt.close(fig)

fig,ax=plt.subplots(figsize=(9,9));ax.set_aspect('equal');ax.set_xlim(-270,270);ax.set_ylim(-90,500)
ax.add_collection(PolyCollection(cells,facecolors='#c7e6df',edgecolors='none',antialiaseds=False,zorder=1))
pts=probe['board_points']
for yy in range(96,417,40):ax.plot([-180,180],[yy,yy],color='#ac986b',lw=.6,zorder=2)
for xx in range(-180,181,40):ax.plot([xx,xx],[96,416],color='#ac986b',lw=.6,zorder=2)
for goodval,color,marker,label in [(True,'#167b6c','o','存在无碰撞逆解'),(False,'#c94b40','x','两个逆解均碰撞')]:
    p=[p for p in pts if p['free']==goodval]
    ax.scatter([v['x'] for v in p],[v['y'] for v in p],c=color,marker=marker,s=45,label=f'{label}：{len(p)} 点',zorder=4)
ax.plot(0,0,'ko');ax.annotate('肩轴 O',(0,0),xytext=(15,-15));ax.grid(color='#ddd',lw=.5)
ax.set_xlabel('X / mm');ax.set_ylabel('Y / mm');ax.legend(loc='upper left',fontsize=10)
ax.set_title('撤回旧摆位：320 × 360 mm 网格，近边96 mm\n用 SolidWorks 实体干涉逐点复核两支逆解',fontsize=14,pad=15)
fig.text(.1,.03,'圆环内有逆解 ≠ 结构无碰撞。这里只核验原候选摆位，不推断所有摆法都不可用。',fontsize=10,color='#b7443d')
fig.subplots_adjust(bottom=.12,top=.89)
for ext in ('png','svg'):fig.savefig(OUT/f'old-board-collision-check.{ext}',dpi=170)
plt.close(fig)
print('rendered',len(cells),'configuration cells')
