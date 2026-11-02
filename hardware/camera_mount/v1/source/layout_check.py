"""创建含原机械臂的静态参考场景；参考棋盘/相机不是打印件。"""
import hashlib
import json
from pathlib import Path
import assemble_mount as a
import build_mount as b


def save_reference(doc,name):
    if doc.SaveAs3(str(b.PARTS/(name+'.SLDPRT')),0,1)!=0:raise RuntimeError('参考件保存失败')


def board():
    d,p=b.new_part('reference_board')
    b.feature(d,p,'520x480棋盘与码区域',[b.rect(-260,-240,260,240)],3,-3)
    b.feature(d,p,'十条横线',[b.rect(x-.2,-160,x+.2,160) for x in range(-180,181,40)],.2)
    shapes=[]
    for y in range(-160,161,40):
        if abs(y)==160:shapes.append(b.rect(-180,y-.2,180,y+.2))
        else:shapes += [b.rect(-180,y-.2,-20,y+.2),b.rect(20,y-.2,180,y+.2)]
    b.feature(d,p,'九列和河界',shapes,.2)
    b.feature(d,p,'四块48mm角标位置',[b.rect(x-24,y-24,x+24,y+24) for x in (-228,228) for y in (-208,208)],.3)
    save_reference(d,'reference_board')


def camera():
    # 只用于安装方向/标高示意。孔阵到PCB边缘、光轴偏置尚需实物确认。
    d,p=b.new_part('reference_camera_B')
    b.feature(d,p,'25x24PCB示意',[b.rect(-12.5,-12,12.5,12)],1)
    b.feature(d,p,'14mm镜头前端外形示意',[b.circle(0,0,7)],16.6,1)
    save_reference(d,'reference_camera_B')


def main():
    project=b.ROOT.parents[2]
    info=json.loads((project/'research/scara-kinematics-geometry.json').read_text(encoding='utf-8-sig'))
    original=Path(info['assembly'])
    original_doc=next(d for d in b.SW.GetDocuments if d.GetPathName.lower()==str(original).lower())
    before=hashlib.sha256(original.read_bytes()).hexdigest()
    flag_before=bool(original_doc.GetSaveFlag)
    board();camera()
    d=a.assembly()
    a.add(d,'camera_mount',[0,0,0],source_path=b.ROOT/'camera_mount.SLDASM',label='全打印相机支架')
    a.add(d,'reference_board',[0,250,0],label='参考棋盘_不打印')
    a.add(d,'reference_camera_B',[0,250,664],[1,0,0,0,-1,0,0,0,-1],label='相机外形示意_光轴偏置待测')
    a.add(d,'original_scara',[.508123,-155.215866,-986],[-1,0,0,0,-1,0,0,0,1],
          source_path=original,label='原机械臂_只读引用')
    a.save(d,'layout_check')
    after=hashlib.sha256(original.read_bytes()).hexdigest()
    assert before==after
    # 插件/引用解析可能改变内存中的待保存标记。只记录，不保存、重载或关闭原装配。
    record={'original_assembly':str(original),'original_sha256_before':before,'original_sha256_after':after,
            'original_unsaved_flag_before':flag_before,'original_unsaved_flag_after':bool(original_doc.GetSaveFlag),
            'reference_components':a.placed,'scope':'只检查名义安装布局；不是全部关节行程扫掠或打印承载验收。棋盘面按底脚安装面同高示意。原装配内存标记仅记录，不自动处理。'}
    (b.ROOT/'layout-reference.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    print('LAYOUT SAVED; ORIGINAL UNCHANGED',flush=True)


if __name__=='__main__':main()
