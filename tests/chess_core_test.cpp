#include "chess/board.hpp"
#include <iostream>
#include <stdexcept>

static void check(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}

int main()
{
    try
    {
        chess::Board initial, after;
        chess::initialBoard(initial);
        chess::Move move;
        std::string text, error;
        chess::fen(initial, true, 7, 15, text);
        check(text == "rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 7 15",
              "FEN order or counters changed");
        check(!chess::uci(move, text) && text.empty(), "Invalid move retained output text");
        check(chess::parseMove("a3a4", move) && chess::uci(move, text) && text == "a3a4", "Move conversion failed");
        check(chess::applyMove(initial, true, move, after, error) && after[6][0] == '.' && after[5][0] == 'P',
              "Move changed the wrong squares");
        //盘面差分仅推演匹配走法，完整90格比较仍需排除额外移动与错认
        chess::Observation seen{after,true};
        check(chess::inferMove(initial,seen,true,{"a3a4","c3c4"},move,error) && move.fromCol==0 && move.toRow==5,
              "Board difference did not identify the moved piece");
        seen.board[6][2]='.';
        seen.board[5][2]='P';
        check(!chess::inferMove(initial,seen,true,{"a3a4","c3c4"},move,error),"Two moved pieces became one move");
        seen.board=after;
        seen.board[5][0]='C';
        check(!chess::inferMove(initial,seen,true,{"a3a4"},move,error),"Changed piece identity accepted");
        seen.board=after;
        check(!chess::inferMove(initial,seen,true,{"a3a4","a3a4"},move,error),"Repeated matching move accepted");
        chess::Board invalid = initial;
        invalid[0][0] = '?';
        check(!chess::validBoard(invalid, error), "Invalid board was accepted at the input boundary");
        check(!chess::parseMove("a3a3", move) && !chess::parseMove("j3a4", move), "Invalid UCI coordinates accepted");
        check(chess::parseMove("e0e3", move) && !chess::applyMove(initial, true, move, after, error),
              "A king outside its palace was accepted");

        chess::Game game;
        check(game.board == initial && game.history.empty(),
              "Game default initialization changed");
        check(chess::Game_judge(game, {}) && game.result == chess::Result::BlackWon,
              "No legal move did not end the current side's game");
        check(chess::Game_reset(game, initial) && chess::Game_finish(game, chess::Result::Draw) &&
                  game.result == chess::Result::Draw,
              "Reset/draw state failed");
        check(chess::Game_reset(game, initial) && chess::Game_finish(game, chess::Result::RedWon) &&
                  game.result == chess::Result::RedWon,
              "Resignation assigned the wrong winner");
        chess::Game_reset(game, initial);
        auto saved = game.board;
        check(!chess::Game_reset(game, invalid) && game.board == saved,
              "Invalid reset replaced the authoritative board");
        check(chess::Game_reset(game,after) && chess::Game_reset(game,game.board) && game.board==after &&
                  game.positions.size()==1 && game.history.empty(),"In-place reset lost the confirmed board");
        std::cout << "PASS FEN/move outputs, rule-data checks, game state, reset and outcomes\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
