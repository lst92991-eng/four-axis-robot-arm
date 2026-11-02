# 通过本次验证过的以太网SSH连接，打开皮卡鱼UCI交互控制台。
# 依次输入 uci、setoption name Hash value 4、isready、position startpos、go movetime 3000。
# 输入 stop 停止搜索，输入 quit 退出。
$ErrorActionPreference = 'Stop'
& ssh -t -b 192.168.55.1 `
    -i C:/Users/lst/.ssh/rv1106_codex_auto `
    -o BatchMode=yes -o StrictHostKeyChecking=yes `
    -o "UserKnownHostsFile=$PSScriptRoot/network/known_hosts" `
    root@192.168.55.6 `
    'cd /root/scara-pikafish-test/2023-03-05 && ulimit -v 140000 && exec ./pikafish'
