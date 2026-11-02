import ast,struct,zipfile,math,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
def npy(data):
    size=struct.unpack_from('<H' if data[6]==1 else '<I',data,8)[0]
    start=10 if data[6]==1 else 12
    h=ast.literal_eval(data[start:start+size].decode('latin1'))
    return h,data[start+size:]
with zipfile.ZipFile(ROOT/'output/constrained-workspace-2026-10-09/cad-collision-scan.npz') as z:
    h,free=npy(z.read('connected.npy'))
assert h['shape']==(91,91) and h['descr']=='|b1' and not h['fortran_order']
rows=[]
for r in range(10):
    for c in range(9):
        x=(c-4)*35;y=280+(r-4.5)*35
        cs=(x*x+y*y-195**2-275**2)/(2*195*275)
        branches=[]
        if abs(cs)<=1:
            for sign in [-1,1]:
                q2=sign*math.acos(cs)
                q1=math.atan2(y,x)-math.atan2(275*math.sin(q2),195+275*math.cos(q2))-math.pi/2
                a=(math.degrees(q1)+180)%360-180;b=math.degrees(q2)
                i=int(math.floor((a+180)/4));j=int(math.floor((b+180)/4))
                supported=0<=i<90 and 0<=j<90 and all(free[(i+di)*91+j+dj] for di in [0,1] for dj in [0,1])
                branches.append({'q1_deg':a,'q2_deg':b,'supported':bool(supported)})
        rows.append({'row_from_robot':r,'col':c,'x_mm':x,'y_mm':y,'supported':any(b['supported'] for b in branches),'branches':branches})
report={'center_world_mm':[0,280],'board_world_rotation_deg':180,'black_side':'near robot','pitch_mm':35,
        'grid_span_mm':[280,315],'supported_points':sum(p['supported'] for p in rows),'total_points':90,
        'source':'old-structure 4-degree CAD collision grid, same original screws, not modified 3.82mm hardware',
        'scope':'discrete kinematic endpoint screening; no new continuous-motion collision or physical validation',
        'points':rows}
(Path(__file__).parent/'front-layout-check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf-8')
print('supported',report['supported_points'],'of 90')
print('unsupported',[(p['x_mm'],p['y_mm']) for p in rows if not p['supported']])
