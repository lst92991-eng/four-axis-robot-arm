"""基于CAD只读投影，计算中国象棋摆位；所有长度为毫米。"""
import json, math, site
from pathlib import Path
site.addsitedir('C:/Users/lst/AppData/Local/Temp/scara-workspace-python')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Polygon
from matplotlib.font_manager import FontProperties

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output/workspace-projection-2026-10-09'
DATA=json.loads((OUT/'cad-projection-evidence.json').read_text('utf-8'))
plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,
    'font.size':11,'axes.titleweight':'bold','svg.fonttype':'path','figure.facecolor':'#f8fafc'})
INK='#172b42'; GREEN='#098779'; BLUE='#347fc4'; RED='#ca5050'
R=470; r=80; W=360; D=320; gap=96

def base(ax,limits=(-510,510,-510,510)):
    ax.set_aspect('equal');ax.set_xlim(limits[:2]);ax.set_ylim(limits[2:])
    ax.set_facecolor('white');ax.grid(color='#dce4eb',lw=.55,zorder=0)
    ax.set_xlabel('横向 X / mm');ax.set_ylabel('前方 Y / mm')
    ax.add_patch(Circle((0,0),R,color='#d8eee9',zorder=1))
    ax.add_patch(Circle((0,0),R,fill=False,color=GREEN,lw=1.8,zorder=2))
    ax.add_patch(Circle((0,0),r,facecolor='#fae5e3',edgecolor=RED,hatch='///',lw=1,zorder=2))
    ax.axvline(0,color='#899bac',lw=.7,ls='--',zorder=3)
    ax.plot(0,0,'o',color=INK,ms=6,zorder=7)
    for spine in ax.spines.values():spine.set_color('#cbd5df')

def cad(ax,alpha=.9):
    for c in sorted(DATA['components'],key=lambda c:c['zmax_mm']):
        geo=c['projection'];kind=geo['type']
        polys=[geo['coordinates']] if kind=='Polygon' else geo['coordinates'] if kind=='MultiPolygon' else []
        for p in polys:
            ax.add_patch(Polygon(p[0],closed=True,facecolor='#6d7d8d',edgecolor='#34485d',lw=.3,alpha=alpha,zorder=4))

def dim(ax,a,b,label,offset=(0,0)):
    ax.annotate('',xy=a,xytext=b,arrowprops={'arrowstyle':'<->','color':INK,'lw':1},zorder=9)
    mid=((a[0]+b[0])/2+offset[0],(a[1]+b[1])/2+offset[1])
    ax.text(*mid,label,ha='center',va='center',color=INK,fontsize=10,
        bbox={'facecolor':'white','edgecolor':'none','pad':2},zorder=10)

def grid(ax,w,d,g,landscape=True,pieces=False):
    xs=np.linspace(-w/2,w/2,10 if landscape else 9)
    ys=np.linspace(g,g+d,9 if landscape else 10)
    ax.add_patch(Rectangle((-w/2,g),w,d,facecolor='#fff5d9',edgecolor='#af8131',lw=1.5,zorder=3))
    for x in xs:ax.plot([x,x],[g,g+d],color='#b79452',lw=.65,zorder=4)
    for y in ys:ax.plot([-w/2,w/2],[y,y],color='#b79452',lw=.65,zorder=4)
    pts=np.array([(x,y) for y in ys for x in xs])
    rr=np.linalg.norm(pts,axis=1);good=(rr>=r)&(rr<=R)
    ax.scatter(pts[good,0],pts[good,1],s=12,color=INK,zorder=6)
    if np.any(~good):ax.scatter(pts[~good,0],pts[~good,1],s=24,color=RED,zorder=6)
    if pieces:
        for p in pts:ax.add_patch(Circle(p,17.5,fill=False,edgecolor='#bb9a63',alpha=.5,lw=.45,zorder=5))
    return pts,rr

fig,(a,b)=plt.subplots(1,2,figsize=(15,9),gridspec_kw={'width_ratios':[1.12,1]})
fig.suptitle('SCARA 俯视工作区与中国象棋摆位',fontsize=23,color=INK,x=.07,ha='left',y=.965)
fig.text(.07,.918,'CAD 实测轴距 195 + 275 mm  ·  棋子 Ø35 mm  ·  交点间距 40 mm  ·  2026-10-09',color='#56687a',fontsize=11)
base(a);cad(a)
a.set_title('① 整机当前姿态投影 + 腕轴理想可达区',fontsize=13,pad=14)
a.plot([0,0,0],[0,195,470],'-o',color='#f9ba42',ms=6,lw=2.5,zorder=7)
a.text(42,285,'275 mm',color=INK,fontsize=10)
a.text(42,100,'195 mm',color=INK,fontsize=10)
a.annotate('肩轴 O（测距原点）',xy=(0,0),xytext=(-450,75),arrowprops={'arrowstyle':'->','color':INK},fontsize=10,zorder=8)
a.annotate('后方立柱 / 底座',xy=(0,-155),xytext=(-450,-200),arrowprops={'arrowstyle':'->','color':INK},fontsize=10,zorder=8)
a.text(-435,-335,'绿色：80 ≤ 腕轴半径 ≤ 470 mm\n红色：中心盲区 R < 80 mm\n灰色：当前姿态的 CAD 实体投影',fontsize=10,color=INK,
    bbox={'facecolor':'white','edgecolor':'#d4e0e8','pad':8},zorder=8)
dim(a,(-470,-485),(470,-485),'外径 940 mm',(0,0))
base(b,(-270,270,-125,510))
b.set_title('② 建议布局：长边横放，短边向前',fontsize=13,pad=14)
pts,dist=grid(b,W,D,gap,True,True)
b.plot(0,256,'+',color=GREEN,ms=15,mew=2,zorder=9)
b.text(0,272,'网格中心 (0, 256)',ha='center',fontsize=10,color=GREEN,zorder=10,
    bbox={'facecolor':'#fff5d9','edgecolor':'none','pad':1})
b.text(0,28,'肩轴 O',ha='center',fontsize=10,zorder=9)
dim(b,(-180,447),(180,447),'360 mm：10 行之间的 9 个间隔')
dim(b,(222,96),(222,416),'320 mm',(0,0))
dim(b,(-224,0),(-224,96),'96 mm',(-7,0))
b.annotate('近端交点：R = 98.1 mm',xy=(-20,96),xytext=(-230,-82),fontsize=10,
    arrowprops={'arrowstyle':'->','color':INK},zorder=9)
b.annotate('远角：R = 453.3 mm',xy=(180,416),xytext=(25,484),fontsize=10,
    arrowprops={'arrowstyle':'->','color':INK},zorder=9)
fig.text(.07,.072,'图中投影沿竖直 Z 轴向下；+Y 指向原装配伸臂前方。圆环中心是肩旋转轴，不是后方立柱中心。',fontsize=10,color=INK)
fig.text(.07,.041,'范围口径：同轴工具的腕轴中心、无关节限位的理想几何；尚未扣除自碰撞、支架、线缆及实际 Z 高度限制。',fontsize=10,color=RED)
fig.subplots_adjust(left=.07,right=.965,top=.865,bottom=.15,wspace=.25)
for ext in ('png','svg'):fig.savefig(OUT/f'workspace-and-board.{ext}',dpi=180)
plt.close(fig)

square=(-2*r+math.sqrt(5*R*R-r*r))/2.5
fig,axs=plt.subplots(1,3,figsize=(15,7))
fig.suptitle('为什么要横放，以及“正方形棋盘”的两种含义',fontsize=20,color=INK,y=.96)
for ax in axs:base(ax,(-270,270,-110,530))
grid(axs[0],360,320,96,True)
axs[0].set_title('推荐：360 横向 × 320 前后',fontsize=12)
axs[0].text(0,-85,'近边允许 80～114.2 mm\n按整个网格矩形留在圆环内计算',ha='center',fontsize=10)
grid(axs[1],320,360,80,False)
axs[1].set_title('不推荐：320 横向 × 360 前后',fontsize=12)
axs[1].text(0,-85,'近边仅允许 80～81.9 mm\n理想状态也几乎没有摆放余量',ha='center',fontsize=10)
axs[2].add_patch(Rectangle((-200,56),400,400,fill=False,edgecolor=BLUE,ls='--',lw=2,zorder=6))
grid(axs[2],360,320,96,True)
axs[2].set_title('若外板为正方形：示例 400 × 400',fontsize=12)
axs[2].text(0,-85,'纸板边角可超出腕轴圆环\n90 个落子交点必须可达',ha='center',fontsize=10)
fig.text(.06,.08,f'若要求一个正方形内每一点都可达：正前方、居中、边线平行摆放时，理论最大边长为 {square:.1f} mm；贴边无余量。',fontsize=10,color=INK)
fig.text(.06,.04,'400 mm 外板仅为布局示例，不是已购棋盘尺寸；象棋落子网格仍为 320 × 360 mm，不能改成正方形网格。',fontsize=10,color=RED)
fig.subplots_adjust(left=.06,right=.97,top=.88,bottom=.18,wspace=.28)
for ext in ('png','svg'):fig.savefig(OUT/f'orientation-comparison.{ext}',dpi=180)
plt.close(fig)

# 对90个交点进行双分支IK与FK回代，校验几何结论。
errors=[];q2s=[]
for x,y in pts:
    c=(x*x+y*y-195**2-275**2)/(2*195*275)
    assert -1<=c<=1
    for sign in (-1,1):
        q2=sign*math.acos(c)
        q1=math.atan2(y,x)-math.atan2(275*math.sin(q2),195+275*math.cos(q2))
        xr=195*math.cos(q1)+275*math.cos(q1+q2)
        yr=195*math.sin(q1)+275*math.sin(q1+q2)
        errors.append(math.hypot(x-xr,y-yr));q2s.append(abs(math.degrees(q2)))
result={'user_confirmed':{'piece_diameter_mm':35,'pitch_xy_mm':[40,40],'columns':9,'rows':10,'grid_span_mm':[320,360]},
    'cad_link_mm':[195,275],'ideal_radius_mm':[80,470],
    'recommended_candidate':{'grid_x_mm':[-180,180],'grid_y_mm':[96,416],'center_mm':[0,256],
        'near_far_point_radius_mm':[float(dist.min()),float(dist.max())],
        'point_radial_margins_mm':[float(dist.min()-r),float(R-dist.max())],
        'continuous_rectangle_margins_mm':[96-r,float(R-dist.max())],
        'absolute_q2_degrees':[min(q2s),max(q2s)],'max_fk_roundtrip_error_mm':max(errors),'reachable_points':90},
    'centered_front_rectangle_near_edge_intervals_mm':{'landscape':[80,math.sqrt(R*R-180*180)-320],'portrait':[80,math.sqrt(R*R-160*160)-360]},
    'centered_front_square_entire_area_ideal_max_side_mm':square,
    'unknown':['真实关节限位','折臂自碰撞及路径','工具TCP与工作高度','成品棋盘外尺寸','线缆与相机支架'],
    'validation_scope':'只验证无约束2R平面几何；没有实物动作、碰撞验收或Z可达性证明。'}
(OUT/'placement-analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
