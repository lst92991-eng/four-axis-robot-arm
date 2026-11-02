"""用CAD实体投影说明已被SolidWorks干涉API确认的两处碰撞。"""
import sys,site,json,math
from pathlib import Path
site.addsitedir('C:/Users/lst/AppData/Local/Temp/scara-workspace-python')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'output/constrained-workspace-2026-10-09'
data=json.loads((ROOT/'output/workspace-projection-2026-10-09/cad-projection-evidence.json').read_text('utf-8'))
groups=json.loads((OUT/'rigid-groups.json').read_text('utf-8'))['groups']
plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'svg.fonttype':'path','figure.facecolor':'#f8fafc'})
def turn(p,angle,origin=(0,0)):
    a=math.radians(angle);r=np.array([[math.cos(a),math.sin(a)],[-math.sin(a),math.cos(a)]])
    return (np.array(p)-origin)@r+origin
fig,axs=plt.subplots(1,2,figsize=(14,7))
for ax,q1,q2,red,blue in [(axs[0],98,0,['M6内六角x22-5'],['J1机械臂-2']),
                         (axs[1],0,160,['L型支架底座-1'],['第二机械臂-1','42步进电机L40-1'])]:
    for c in data['components']:
        name=c['name']
        if name not in groups:continue
        geom=c['projection'];ps=[geom['coordinates']] if geom['type']=='Polygon' else geom['coordinates'] if geom['type']=='MultiPolygon' else []
        highlight=name in red or name in blue
        for p in ps:
            xy=np.array(p[0]);g=groups[name]
            if g==2:xy=turn(xy,q2,(0,195))
            if g>0:xy=turn(xy,q1)
            ax.add_patch(Polygon(xy,facecolor='#d04945' if name in red else '#348ecc' if name in blue else '#b4bec6',
                edgecolor='#223a50' if highlight else '#98a5af',alpha=.8 if highlight else .2,lw=.65 if highlight else .3,zorder=5 if highlight else 1))
    ax.plot(0,0,'ko',ms=5);ax.text(5,4,'肩轴');ax.set_aspect('equal');ax.grid(color='#dce4eb',lw=.6)
    ax.set_xlabel('X / mm');ax.set_ylabel('Y / mm');ax.set_facecolor('white')
axs[0].set_xlim(-270,100);axs[0].set_ylim(-170,100)
axs[0].set_title('证据 A：J1 转向98°，臂板碰到M6螺钉',fontsize=13,pad=15)
axs[1].set_xlim(-200,100);axs[1].set_ylim(-180,240)
axs[1].set_title('证据 B：J1=0°、J2=160°，第二臂碰支架底座',fontsize=13,pad=15)
fig.suptitle('机械结构确实截断了理想圆环',fontsize=22,x=.07,ha='left',y=.96)
fig.text(.07,.045,'红色：碰撞对象；蓝色：转动件。图为CAD俯视投影；实体相交由SolidWorks三维干涉API确认，不是仅凭投影重叠判断。',fontsize=10)
fig.subplots_adjust(left=.07,right=.96,top=.85,bottom=.12,wspace=.25)
for ext in ('png','svg'):fig.savefig(OUT/f'collision-witnesses.{ext}',dpi=180)
print('saved collision witnesses')
