"""统一比例绘制实体碰撞采样结果；仅离线绘图。"""
import json,site,math,sys
from pathlib import Path
import numpy as np
site.addsitedir('C:/Users/lst/AppData/Local/Temp/scara-workspace-python')
sys.path.insert(0,'C:/Users/lst/AppData/Local/Temp/scara-workspace-comparison-deps')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle,Patch
from matplotlib.lines import Line2D
from shapely.geometry import Polygon
from shapely.ops import unary_union
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'output/workspace-comparison-2026-10-13'
plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'font.size':11,'svg.fonttype':'path','figure.facecolor':'#f5f7f8'})
colors={'old_head6':'#587c9a','old_head3p82':'#2b9d8f','new_head6':'#587c9a','new_head3p82':'#2b9d8f'}
data={};geometry={};metrics={}
for key in colors:
    report=json.loads((OUT/(key+'.json')).read_text('utf-8'));d=np.load(OUT/(key+'.npz'))
    a,b=np.meshgrid(np.deg2rad(d['angles']),np.deg2rad(d['angles']),indexing='ij');l1,l2=report['links_mm']
    x=-l1*np.sin(a)-l2*np.sin(a+b);y=l1*np.cos(a)+l2*np.cos(a+b);mask=d['connected'];polys=[]
    for i in range(len(mask)-1):
        for j in range(len(mask)-1):
            if np.all(mask[i:i+2,j:j+2]):
                p=Polygon([(x[i,j],y[i,j]),(x[i+1,j],y[i+1,j]),(x[i+1,j+1],y[i+1,j+1]),(x[i,j+1],y[i,j+1])]).buffer(0)
                if not p.is_empty:polys.append(p)
    shape=unary_union(polys);radius=np.hypot(x[mask],y[mask]);geometry[key]=shape;data[key]=report
    metrics[key]={'links_mm':[l1,l2],'head_height_mm':report['head_height_mm'],'sampled_field_area_cm2':float(shape.area/100),'min_sampled_radius_mm':float(radius.min()),'max_sampled_radius_mm':float(radius.max()),'board_free_endpoints':report['board']['free_endpoint_count'],'total_board_points':90,'first_axis_contact_boundary':report['first_axis_contact_boundary']}
def fill(ax,geom,color,alpha=1,z=2):
    from matplotlib.path import Path as MPath
    from matplotlib.patches import PathPatch
    for p in ([geom] if geom.geom_type=='Polygon' else getattr(geom,'geoms',[])):
        if p.geom_type!='Polygon':continue
        rings=[p.exterior,*p.interiors];v=[];codes=[]
        for ring in rings:
            xy=np.asarray(ring.coords);v.extend(xy);codes.extend([MPath.MOVETO]+[MPath.LINETO]*(len(xy)-2)+[MPath.CLOSEPOLY])
        ax.add_patch(PathPatch(MPath(np.asarray(v),codes),facecolor=color,edgecolor='none',alpha=alpha,zorder=z))
def board(ax,report):
    ax.add_patch(Rectangle((-157.5,105),315,350,facecolor='#e9cf9a',alpha=.32,edgecolor='#684e2c',lw=1.4,zorder=4))
    for x in np.arange(-140,141,35):ax.plot([x,x],[122.5,437.5],color='#695438',lw=.5,zorder=5)
    for y in np.arange(122.5,438,35):ax.plot([-140,140],[y,y],color='#695438',lw=.5,zorder=5)
    for x,y in [(-195,67.5),(195,67.5),(-195,492.5),(195,492.5)]:
        ax.add_patch(Rectangle((x-30,y-30),60,60,fill=False,edgecolor='#7e786b',ls=':',lw=1,zorder=4))
    points=report['board']['points'];bad=[p for p in points if not p['free']]
    if bad:ax.scatter([p['x'] for p in bad],[p['y'] for p in bad],marker='x',c='#c93a43',s=50,lw=1.8,zorder=8)
    ax.text(0,293,'棋盘正前方摆位\n315×350 mm · 格距35 mm\n棋子直径33 mm',ha='center',va='center',fontsize=9,color='#3d3224',bbox={'facecolor':'#fff8e6','alpha':.88,'edgecolor':'none','pad':3},zorder=9)
    # 以实际33mm棋子显示首末两排；可达采样范围仍指腕轴中心。
    for y in (122.5,437.5):
        for x in np.arange(-140,141,35):ax.add_patch(Circle((x,y),16.5,fill=False,edgecolor='#5a4630',lw=.55,zorder=6))
def setup(ax):
    ax.set_aspect('equal');ax.set_xlim(-590,590);ax.set_ylim(-590,590);ax.set_facecolor('white')
    ax.set_xticks(np.arange(-500,501,250));ax.set_yticks(np.arange(-500,501,250));ax.grid(color='#dce3e7',lw=.6,zorder=0)
    ax.set_xlabel('左右 X / mm');ax.set_ylabel('前方 Y / mm')
    ax.add_patch(Rectangle((-60,-214),120,160,facecolor='#4f5966',alpha=.85,zorder=6))
    ax.plot(0,0,'o',color='#182f41',ms=5,zorder=8);ax.text(15,-28,'肩轴 O',fontsize=9,zorder=8)
    ax.text(0,-137,'立柱底座',color='white',ha='center',fontsize=8,zorder=8)
    for spine in ax.spines.values():spine.set_color('#b8c8d2')
def panel(ax,key,title):
    r=data[key];l1,l2=r['links_mm'];setup(ax)
    ax.add_patch(Circle((0,0),l1+l2,facecolor='#f0e6e3',edgecolor='#bbaba8',ls='--',lw=1,zorder=1))
    fill(ax,geometry[key],colors[key]);board(ax,r)
    ax.set_title(title,fontsize=15,pad=14,color='#19364a')
    m=metrics[key]
    ax.text(-550,-535,f'采样投影约 {m["sampled_field_area_cm2"]:.0f} cm²\n棋盘端点通过 {m["board_free_endpoints"]}/90',fontsize=10,color='#19364a',bbox={'facecolor':'white','edgecolor':'#d9e2e7','pad':6},zorder=10)
def pair(keys,title,subtitle,filename):
    fig,axes=plt.subplots(1,2,figsize=(15,9))
    fig.suptitle(title,x=.065,ha='left',y=.965,fontsize=22,color='#19364a')
    fig.text(.065,.918,subtitle,fontsize=11,color='#536a79')
    for ax,key in zip(axes,keys):
        r=data[key];panel(ax,key,f'{"旧版" if key.startswith("old") else "新版"} {r["links_mm"][0]}＋275 mm ｜ 螺丝头 {r["head_height_mm"]} mm')
    legend=[Patch(color=colors[keys[-1]],label='实体干涉采样支持区域'),Patch(color='#f0e6e3',label='理论圆内未支持区域'),Line2D([0],[0],color='#c93a43',marker='x',ls='',label='未通过的棋盘端点')]
    fig.legend(handles=legend,loc='lower center',bbox_to_anchor=(.5,.12),ncol=3,frameon=False)
    fig.text(.065,.075,'统一口径：204个实体；当前Z高度和腕姿态；均排除传动子总成。范围指腕轴中心，虚线为理论最大半径。',fontsize=10)
    fig.text(.065,.04,'4°离散采样与投影插值；棋盘点另做精确端点干涉检查。尚未验证连续路径、电磁铁、线缆和实机误差。',fontsize=10,color='#a3453e')
    fig.subplots_adjust(left=.065,right=.97,bottom=.21,top=.86,wspace=.20)
    for ext in ('png','svg'):fig.savefig(OUT/f'{filename}.{ext}',dpi=180)
    plt.close(fig)
pair(['old_head6','new_head6'],'新旧机械臂可达工作区 · 俯视对比','与两份原生总装一致：同为6 mm原螺丝头、同一棋盘位置、同一比例；G1由195增至275 mm。','old-new-top-view')
pair(['old_head3p82','new_head3p82'],'换低头后 · 新旧机械臂俯视对比','同一棋盘位置、同一比例、同为3.82 mm低头螺丝分析变体；G1由195增至275 mm。','old-new-low-heads-top-view')
pair(['old_head6','old_head3p82'],'旧版机械臂 · 更换低头螺丝前后','195＋275 mm臂长保持不变；只将两颗限位M6头部从6降至3.82 mm，承压面与杆部不移动。','screw-change-top-view')
fig,axes=plt.subplots(1,2,figsize=(15,9))
fig.suptitle('新版275＋275 mm · 两颗M6螺丝改薄前后',x=.065,ha='left',y=.967,fontsize=22,color='#19364a')
fig.text(.065,.918,'同一机械臂、同一坐标和比例；承压面与杆部不动，只将头高6 mm改为3.82 mm。绿色突出新增范围。',fontsize=11,color='#536a79')
panel(axes[0],'new_head6','改薄前｜M6头高6 mm')
panel(axes[1],'new_head3p82','改薄后｜M6头高3.82 mm')
before=geometry['new_head6'];after=geometry['new_head3p82']
fill(axes[1],after.intersection(before),'#587c9a',z=2)
fill(axes[1],after.difference(before),'#36aa78',z=2)
gain=float((after.area-before.area)/100);percent=float((after.area/before.area-1)*100)
axes[1].text(0,-420,f'新增约{gain:.0f} cm²（＋{percent:.1f}%）',ha='center',fontsize=11,color='#145c3d',bbox={'facecolor':'white','edgecolor':'#36aa78','pad':6},zorder=10)
fig.legend(handles=[Patch(color='#587c9a',label='原有采样支持范围'),Patch(color='#36aa78',label='改薄后新增范围'),Patch(color='#f0e6e3',label='理论圆内未支持区域')],loc='lower center',bbox_to_anchor=(.5,.12),ncol=3,frameon=False)
fig.text(.065,.076,'肩部首次基座干涉：约±97.74° → ±114.61°；之后仍受角座约束。最大半径550 mm不变，两者棋盘端点均90/90。',fontsize=10)
fig.text(.065,.04,'204实体、当前Z/R、4°采样；均排除传动子总成。低头为截低原头的分析假设；不是连续路径或实机验收。',fontsize=10,color='#a3453e')
fig.subplots_adjust(left=.065,right=.97,bottom=.21,top=.86,wspace=.20)
for ext in ('png','svg'):fig.savefig(OUT/f'new-arm-screw-change-top-view.{ext}',dpi=180)
plt.close(fig)
fig,axes=plt.subplots(2,2,figsize=(14,15))
fig.suptitle('臂长与螺丝头高度 · 四种组合',fontsize=22,color='#19364a',y=.975)
for ax,key in zip(axes.flat,['old_head6','new_head6','old_head3p82','new_head3p82']):
    r=data[key];panel(ax,key,f'{r["links_mm"][0]}＋275 mm ｜ 头高{r["head_height_mm"]} mm')
fig.text(.07,.025,'各图同比例；204实体、4°采样、当前Z/R。低头螺丝为截低原头部的几何假设，非实际商品轮廓。',fontsize=10,color='#a3453e')
fig.subplots_adjust(left=.07,right=.96,bottom=.07,top=.93,hspace=.24,wspace=.24)
for ext in ('png','svg'):fig.savefig(OUT/f'all-four-top-view.{ext}',dpi=160)
plt.close(fig)
for old,new,label in [('old_head6','old_head3p82','lower_heads_old'),('old_head3p82','new_head3p82','long_arm_low_heads')]:
    metrics[label]={'added_area_cm2':float(geometry[new].difference(geometry[old]).area/100),'lost_area_cm2':float(geometry[old].difference(geometry[new]).area/100),'net_area_change_cm2':float((geometry[new].area-geometry[old].area)/100)}
(OUT/'plot-metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2),'utf-8')
print(json.dumps(metrics,ensure_ascii=False,indent=2))
