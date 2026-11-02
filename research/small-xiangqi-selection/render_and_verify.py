"""生成选型摆位图，并做同分支FK回代/轻微摆放扰动初筛；不访问SolidWorks。"""
import json,math,itertools
from pathlib import Path
import numpy as np
from evaluate_layouts import ROOT,OUT,SOURCE,grid,ik,branches,score_centers
import shapely
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.collections import PolyCollection

j=json.loads((OUT/'layout-evaluation.json').read_text('utf-8'))
chosen=[next(r for r in j['practical'] if r['pitch_mm']==32 and r['center_xy_mm']==[290,50]),
        next(r for r in j['optimized'] if r['pitch_mm']==32)]
checks=[]
for r in chosen+[next(r for r in j['practical'] if r['pitch_mm']==30 and r['center_xy_mm']==[290,50])]:
    pts=grid(r['pitch_mm'],r['theta_deg'])+r['center_xy_mm'];branch=0 if r['selected_elbow_branch']==-1 else 1
    a,b,_=ik(pts[:,0],pts[:,1])[branch];aa=np.deg2rad(a);bb=np.deg2rad(b)
    xr=-195*np.sin(aa)-275*np.sin(aa+bb);yr=195*np.cos(aa)+275*np.cos(aa+bb)
    error=float(np.hypot(xr-pts[:,0],yr-pts[:,1]).max());assert error<1e-9
    cases=[]
    for dx,dy,dt in itertools.product([-3,0,3],[-3,0,3],[-.5,0,.5]):
        pp=grid(r['pitch_mm'],r['theta_deg']+dt)+np.array(r['center_xy_mm'])+[dx,dy]
        a,b,valid=ik(pp[:,0],pp[:,1])[branch]
        good=shapely.covers(branches[branch],shapely.points(a,b))&valid
        cases.append({'dx_mm':dx,'dy_mm':dy,'dtheta_deg':dt,'supported':int(good.sum())})
    checks.append({'pitch_mm':r['pitch_mm'],'theta_deg':r['theta_deg'],'center_xy_mm':r['center_xy_mm'],
        'fk_max_error_mm':error,'tested_pose_perturbations':27,'all_90_supported_cases':sum(c['supported']==90 for c in cases),
        'min_supported_points_in_perturbation':min(c['supported'] for c in cases),
        'scope':'只在原4度CAD采样支持域内检查，±3mm/±0.5°是离线扰动输入，不是机器人实测定位精度。','cases':cases})
(OUT/'verification.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2),encoding='utf-8')

z=np.load(SOURCE);angles=np.deg2rad(z['angles']);free=z['connected'];q1,q2=np.meshgrid(angles,angles,indexing='ij')
x=-195*np.sin(q1)-275*np.sin(q1+q2);y=195*np.cos(q1)+275*np.cos(q1+q2)
cells=[]
for i in range(len(angles)-1):
    for k in range(len(angles)-1):
        if free[i:i+2,k:k+2].all():cells.append([(x[i,k],y[i,k]),(x[i+1,k],y[i+1,k]),(x[i+1,k+1],y[i+1,k+1]),(x[i,k+1],y[i,k+1])])
plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'font.size':10,'svg.fonttype':'path','figure.facecolor':'#f8fafc'})
fig,axs=plt.subplots(1,2,figsize=(14,8))
fig.suptitle('小尺寸棋具选型：28 mm棋子 + 32 mm候选格距',fontsize=20,x=.07,ha='left',y=.963)
fig.text(.07,.914,'32 mm是适配设计候选，不是已核实的商品配套棋盘格距；网格交点跨度256 × 288 mm。',fontsize=11,color='#53677c')
titles=['A｜易量尺：中心右290、前50 mm','B｜斜放45°：中心右216、前216 mm']
for ax,r,title in zip(axs,chosen,titles):
    ax.set_aspect('equal');ax.set_xlim(-110,510);ax.set_ylim(-180,470)
    ax.set_facecolor('white');ax.add_collection(PolyCollection(cells,facecolors='#d3ebe6',edgecolors='none',antialiaseds=False,zorder=1))
    pts=grid(r['pitch_mm'],r['theta_deg'])+r['center_xy_mm'];g=pts.reshape(10,9,2)
    for row in g:ax.plot(row[:,0],row[:,1],color='#a28647',lw=.7,zorder=3)
    for col in g.transpose(1,0,2):ax.plot(col[:,0],col[:,1],color='#a28647',lw=.7,zorder=3)
    for px,py in pts:ax.add_patch(Circle((px,py),14,fill=False,edgecolor='#b4a174',lw=.5,zorder=2))
    ax.scatter(pts[:,0],pts[:,1],s=12,color='#153e49',zorder=4)
    ax.plot(0,0,'ko',ms=5);ax.text(10,-20,'肩轴O',fontsize=10)
    ax.plot(*r['center_xy_mm'],'+',color='#d47825',ms=13,mew=2,zorder=5)
    ax.set_title(title,fontsize=12,pad=12);ax.set_xlabel('X / mm（右）');ax.set_ylabel('Y / mm（前）')
    ax.grid(color='#dce4eb',lw=.55,zorder=0)
    ax.text(-85,-135,f'同一肘分支：90/90点初筛支持\n配置空间最小角度支持余量：{r["min_angular_support_margin_deg"]:.1f}°',fontsize=10,
        bbox={'facecolor':'white','edgecolor':'none','pad':5},zorder=5)
fig.text(.07,.07,'浅绿：父聊天已有CAD碰撞工作区采样；圆圈：棋子直径示意，不表示90枚同时摆满。未改变机械臂结构。',fontsize=10)
fig.text(.07,.04,'仅离线尺寸初筛：未验证样本间连续碰撞、抓手/TCP与落子高度、商品实测、吸取成功率或实际轨迹。',fontsize=10,color='#b7443d')
fig.subplots_adjust(left=.07,right=.96,top=.855,bottom=.14,wspace=.23)
for ext in ('png','svg'):fig.savefig(OUT/f'candidate-layouts.{ext}',dpi=170)
print(json.dumps([{k:v for k,v in c.items() if k!='cases'} for c in checks],ensure_ascii=False))
