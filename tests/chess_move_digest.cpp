// 对比全部起终点的数据搬子行为；真正象棋合法性仍由引擎给出。
#include "board.hpp"
#include <cstdint>
#include <iostream>
#include <vector>

int main()
{
    chess::Board initial = chess::initialBoard();
    chess::Board sparse{};
    for (auto &row : sparse)
        row.fill('.');
    sparse[9][4] = 'K';
    sparse[0][4] = 'k';
    sparse[7][0] = 'R';
    sparse[2][0] = 'r';
    chess::Board edge = sparse;
    edge[9][4] = '.';
    edge[0][4] = '.';
    edge[7][3] = 'K';
    edge[2][5] = 'k';
    uint64_t digest = 1469598103934665603ULL;
    auto feed = [&](unsigned char value) { digest = (digest ^ value) * 1099511628211ULL; };
    for (const auto &board : std::vector<chess::Board>{initial, sparse, edge})
        for (bool red : {false, true})
            for (int from = 0; from < 90; ++from)
                for (int to = 0; to < 90; ++to)
                {
                    chess::Move move{from / 9, from % 9, to / 9, to % 9};
                    chess::Board after;
                    bool accepted = chess::validTransfer(board, red, move);
                    if (accepted)
                    {
                        after = board;
                        chess::movePiece(after, move);
                    }
                    feed(accepted);
                    if (accepted)
                        for (const auto &row : after)
                            for (char piece : row)
                                feed(piece);
                    feed(0);
                }
    std::cout << std::hex << digest << '\n';
#ifdef SCARA_EXPECTED_MOVE_DIGEST
    return digest == SCARA_EXPECTED_MOVE_DIGEST ? 0 : 1;
#else
    return 0;
#endif
}
