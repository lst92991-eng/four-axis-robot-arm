"""33mm新旧总装合包，附工作区证据；ZIP逐文件哈希复核。"""
import json,hashlib,zipfile,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'hardware/xiangqi33-assemblies-2026-10-13';A=ROOT/'output/workspace-comparison-2026-10-13'
metrics=json.loads((A/'plot-metrics.json').read_text('utf-8'))
table=['| 臂长 mm | 两颗头高 mm | 采样投影 cm² | 当前棋盘端点通过 |','|---|---:|---:|---:|']
for key in ('old_head6','old_head3p82','new_head6','new_head3p82'):
    r=metrics[key];table.append(f'| {r["links_mm"][0]}＋275 | {r["head_height_mm"]} | {r["sampled_field_area_cm2"]:.0f} | {r["board_free_endpoints"]}/90 |')
report='''# 新旧工作区俯视对比与螺丝改进（2026-10-13）

棋子按用户最新更正采用33 mm；格距仍35 mm、中心(0,280) mm。棋子尺寸不改变本次腕轴中心采样或90交点坐标；本次没有把邻子、电磁铁或实物抓取净空算作已通过。

- `old-new-top-view.png`：两版原6 mm螺丝，与本次两份原生总装一致。
- `new-arm-screw-change-top-view.png`：用户最新指定的新版275＋275，6→3.82mm前后对比；绿色单独标出新增采样范围，约715cm²、10.1%。
- `screw-change-top-view.png`：旧195＋275 mm不变，比较6与3.82 mm头高。
- `old-new-low-heads-top-view.png`：两版同用3.82 mm头高的分析变体。
- `all-four-top-view.png`：全部四种组合。各图坐标、比例、棋盘摆位相同。

'''+ '\n'.join(table)+f'''

旧臂换低头后，采样投影约增加{metrics['lower_heads_old']['net_area_change_cm2']:.0f} cm²（约{metrics['lower_heads_old']['net_area_change_cm2']/metrics['old_head6']['sampled_field_area_cm2']*100:.1f}%），棋盘通过点86增至89。肩部首次基座干涉由约±97.738°放宽到±114.610°；再回收时碰到铝型材角座。近侧中心(0,122.5) mm需要肩角约±118.224°，因此仍未通过。换低头没有消除所有结构限制。

新版275＋275 mm在原6 mm头高时即有90/90端点通过；低头变体也是90/90。只代表本参考摆位下各点至少有一支无碰撞IK端点，不证明整局搬子路径已经通过。

## 范围与方法

两版统一153个顶层零件、204个实体，均排除传动子总成；没有混用历史旧版不含传动、新版含传动的面积口径。5对已记录的轴/轴套/键连接交叠单独排除，其余跨组初始干涉会阻止扫描。

保持当前装配Z高度和腕的相对姿态，肩/肘4°采样，共8,281组构型。图中取直臂所在离散连通分量。投影面积由四角均通过的构型单元经FK投影、合并得到，是近似面积；边界不能直接当实机软限位。所有90交点另解两支IK并做精确端点实体干涉检查。

绿色/蓝色区域表示腕轴中心位置，不是整机实体扫掠外包络，也不代表全部Z高度。尚未纳入传动总成、完整电磁铁、线缆、实际TCP、动态挠曲及装配误差；连续轨迹和真实安全净空仍须复核。

## 低头螺丝的几何假设

仅`M6内六角x22-1`和`M6内六角x22-5`。承压面与杆部不动，把原头从6截低至3.82 mm；用实体极值检查头部切面和杆底。不是把整颗螺丝下移2.18 mm。低头商品的完整头径/圆角/垫片尚未取得，因此原头截低仅作为分析轮廓假设。

原G1底面相对CAD Z=1000约283.7535 mm；承压面277.8535 mm，新头顶281.6735 mm，名义静态间隙约2.08 mm。两份总装仍为原6 mm头高；没有擅自替换实购物料。

四个NPZ与同名JSON保存采样、连通性、实体范围、交点解和首接触角度；`plot-metrics.json`保存近似投影面积。API裁切盒依据[SolidWorks官方CreateBodyFromBox3](https://help.solidworks.com/2016/english/api/sldworksapi/SOLIDWORKS.Interop.sldworks~SOLIDWORKS.Interop.sldworks.IModeler~CreateBodyFromBox3.html)的底面中心与宽/长/高定义，并回读实体极值确认。所有运算只用临时BRep，未保存源CAD。
'''
(A/'对比说明.md').write_text(report,'utf-8');shutil.copytree(A,OUT/'workspace-comparison',dirs_exist_ok=True)
notes='''# 33mm象棋：新旧SCARA参考总装

解压整个压缩包，用SolidWorks 2026分别打开：

- 旧195＋275：`old/SolidWorks/SCARA_FRONT_195_275_CHESS33.SLDASM`
- 新275＋275：`long/SolidWorks/SCARA_FRONT_275_275_CHESS33.SLDASM`

不要只复制SLDASM；保留各自SolidWorks文件夹。每版文件夹另有同名STEP、实际CAD预览和四个独立ArUco打印稿。

## 最新规格

用户2026-10-13把棋子直径30更正为33mm。两版均32枚，红黑14种身份，原生驱动圆直径33mm、高8mm；字面0.1mm凸起。自建参考棋子，不冒充开源商品模型或已实测实物。

参考棋盘315×350×14mm，格距35mm，90交点跨度280×315mm，中心在肩轴前方(0,280)mm；黑方近机械臂。相邻圆饼净距2mm，最外侧圆饼距板边1mm。只改变直径，棋盘及37个棋具/标记变换不变；实际棋高、质量、磁吸和抓手/邻子净空需单独验收。

四码DICT_4X4_50的ID0～3，黑码48mm、白边各6mm、外形60mm；总外包络450×485mm。`old/markers/`与`long/markers/`打印稿一致，打印按100%实际尺寸；`reference/`有用户提供的RV1106 PCB STEP。尚未冻结机位或摄像头支架。

## 已核对

每版独立副本打开、38个顶层/195个总组件；32枚棋子全部回读为33mm，圆柱面回读轴距分别195＋275和275＋275mm；保存引用、运行引用均在本版SolidWorks文件夹内，虚拟皮带和传动装配已外部化。STEP导出与实际CAD预览已检查。

30mm原交付及源275＋275候选结构保留；源文件哈希未变化。原结构既有配合问题保留；本次没有改机械结构，没有实物电机动作、commit/push。按用户已授权处理参考副本CAM保存提示。

## 工作区

`workspace-comparison/old-new-top-view.png`直接比较新旧原结构；另有换低头前后、两版同用低头及四组合图。均按204实体、当前Z/R与4°采样的同一口径。旧机原头86/90，低头分析89/90；新机两种头高均90/90。

**原生总装仍保留6mm螺丝头；3.82mm只作为独立分析变体。** 端点通过不代表连续搬运、电磁铁、线缆、动态挠度或实机验收完成。详见`workspace-comparison/对比说明.md`。

`old/validation.json`和`long/validation.json`保存尺寸/引用/轴距证据，`SHA256SUMS.json`逐文件SHA-256用于完整性核对。
'''
(OUT/'README_先读我.md').write_text(notes,'utf-8')
for label in ('old','long'):
    p=OUT/label/'validation.json';r=json.loads(p.read_text('utf-8'))
    assert r['chess_piece_count']==32 and r['piece_diameter_mm']==33 and not r['outside_stored_references'] and not r['outside_runtime_references']
    assert (OUT/label/'preview/isometric.png').exists() and (OUT/label/'preview/top.png').exists()
    r['opened_independent_copy']=True;r['actual_CAD_previews_visually_checked']=True;p.write_text(json.dumps(r,ensure_ascii=False,indent=2),'utf-8')
baseline=json.loads((OUT/'source-30mm-sha256.json').read_text('utf-8'))
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in baseline.items())
files=[p for p in OUT.rglob('*') if p.is_file() and not p.name.startswith('~$') and p.suffix.lower()!='.bmp' and p.name!='SHA256SUMS.json']
assert all(p.stat().st_size>0 for p in files)
manifest={p.relative_to(OUT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
(OUT/'SHA256SUMS.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),'utf-8');files.append(OUT/'SHA256SUMS.json')
archive=ROOT/'output/SCARA_old-new_33mm_chess_2026-10-13.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for p in files:z.write(p,'SCARA_33mm/'+p.relative_to(OUT).as_posix())
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    for name,h in manifest.items():assert hashlib.sha256(z.read('SCARA_33mm/'+name)).hexdigest()==h
result={'zip':str(archive),'bytes':archive.stat().st_size,'files':len(files),'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'piece_diameter_mm':33,'complete':True}
(ROOT/'output/SCARA_33mm_delivery.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),'utf-8')
print(json.dumps(result,ensure_ascii=False))
