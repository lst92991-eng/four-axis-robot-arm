from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[2]
D=ROOT/'hardware/xiangqi30-old195275-2026-10-12/delivery-front-2026-10-13'
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="900" height="1230" viewBox="0 0 600 820">',
 '<rect width="600" height="820" fill="#f7f5ef"/>',
 '<g font-family="Microsoft YaHei, sans-serif" fill="#263642">',
 '<text x="20" y="30" font-size="17">用户指定：正前方居中摆位</text>',
 '<text x="20" y="52" font-size="10">参数俯视图 · 不是 SolidWorks 截图 · 单位 mm</text>',
 '<rect x="240" y="96" width="120" height="80" rx="3" fill="#cdd4d8" stroke="#657783"/>',
 '<text x="270" y="133" font-size="13">机械臂底座</text>',
 '<line x1="300" y1="176" x2="300" y2="270" stroke="#657783" stroke-width="2"/>',
 '<circle cx="300" cy="250" r="7" fill="#314b61"/>',
 '<text x="312" y="246" font-size="11">肩轴原点 (0,0)</text>',
 '<line x1="300" y1="265" x2="300" y2="310" stroke="#53728b" stroke-width="2"/>',
 '<path d="M295 302 L300 312 L305 302" fill="none" stroke="#53728b"/>',
 '<text x="310" y="292" font-size="11">机械臂正前方 +Y</text>',
 '<rect x="142.5" y="355" width="315" height="350" fill="#e8d1a1" stroke="#9d8257"/>']
for r in range(10):
 y=372.5+r*35;svg.append(f'<path d="M160 {y} H440" fill="none" stroke="#6a5940" stroke-width=".8"/>')
for c in range(9):
 x=160+c*35;svg.append(f'<path d="M{x} 372.5 V687.5" fill="none" stroke="#6a5940" stroke-width=".8"/>')
rows=['rnbakabnr','.........','.c.....c.','p.p.p.p.p','.........','.........','P.P.P.P.P','.C.....C.','.........','RNBAKABNR']
letters=dict(zip('KABNRCPkabnrcp','帅仕相马车炮兵将士象马车炮卒'))
for r,row in enumerate(rows):
 for c,kind in enumerate(row):
  if kind=='.':continue
  x=160+c*35;y=372.5+r*35;color='#b82926' if kind.isupper() else '#202b35'
  svg.append(f'<circle cx="{x}" cy="{y}" r="15" fill="#fff3d7" stroke="{color}"/><text x="{x}" y="{y+5}" text-anchor="middle" font-size="15" fill="{color}">{letters[kind]}</text>')
for x,y in [(-35,122.5),(0,122.5),(35,122.5),(0,157.5)]:
 svg.append(f'<circle cx="{300+x}" cy="{250+y}" r="17" fill="none" stroke="#da523e" stroke-width="2" stroke-dasharray="3 2"/>')
for id,x,y in [(0,195,67.5),(1,-195,67.5),(2,-195,492.5),(3,195,492.5)]:
 svg.append(f'<rect x="{300+x-30}" y="{250+y-30}" width="60" height="60" fill="white" stroke="#526d80"/><text x="{300+x}" y="{250+y+5}" text-anchor="middle" font-size="17">ID {id}</text>')
svg += ['<circle cx="300" cy="530" r="3" fill="#3a6f91"/>',
 '<text x="315" y="527" font-size="11">中心 (0,280)</text>',
 '<text x="194" y="347" font-size="12">黑方靠机械臂</text>',
 '<text x="220" y="725" font-size="12">红方朝向操作者</text>',
 '<text x="20" y="790" font-size="11">红色虚线圈：旧结构采样尚未支持的4点；其余86点有采样支持。</text>',
 '<text x="20" y="809" font-size="10">棋盘315×350 / 格距35 / 棋子直径30；不是连续路径或实机验收。</text>', '</g></svg>']
(D/'front-layout-reference.svg').write_text(''.join(svg),'utf-8')
print(D/'front-layout-reference.svg')
