// 声明板端棋力引擎的查询与收尾接口，走法用棋局的数字行列传递。
#pragma once
#include "board.hpp"
#include <csignal>

//---------UCI 查询接口区---------
// 单局同步使用。先取合法走法，轮到电脑时再向同一局面请求应手。
// thinkMs 使用入口核对过的 1–60000 毫秒。返回 true 时 legal 可用，电脑回合有合法走法时 best 给出应手。
bool Pikafish_query(const char *directory, const chess::Game &game, int thinkMs,
                    const volatile std::sig_atomic_t &running, chess::MoveList &legal, chess::Move &best);
// 收回引擎子进程和通信端，返回是否已经完成收尾。
bool Pikafish_close();
