#!/bin/sh
set -eu
cd "$(dirname "$0")"
echo '鼠标拖动你方棋子；空格暂停/继续，N单步运动，Esc退出。'
echo '参数是机械臂颜色：black表示你执红先走，red表示机械臂执红先走。'
echo 'SolidWorks三维视图：右键旋转，滚轮缩放，R恢复视角。'
exec ./run.sh "${1:-black}" --interactive
