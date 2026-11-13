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
        chess::Board initial, parsed, after;
        chess::initialBoard(initial);
        chess::Move move;
        std::string text, error;
        bool red = false;
        check(chess::fen(initial, true, 7, 15, text), "Initial FEN failed");
        check(text == "rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 7 15",
              "FEN order or counters changed");
        check(chess::parseFen(text, parsed, red, error) && red && parsed == initial, "FEN round trip failed");
        check(!chess::uci(move, text) && text.empty(), "Invalid move retained output text");
        check(chess::parseMove("a3a4", move) && chess::uci(move, text) && text == "a3a4", "Move conversion failed");
        check(chess::applyMove(initial, true, move, after, error) && after[6][0] == '.' && after[5][0] == 'P',
              "Move changed the wrong squares");
        chess::Board invalid = initial;
        invalid[0][0] = '?';
        check(!chess::fen(invalid, true, 0, 1, text) && text.empty(), "Invalid board retained FEN output");
        check(!chess::parseMove("a3a3", move) && !chess::parseMove("j3a4", move), "Invalid UCI coordinates accepted");
        check(chess::parseMove("e0e3", move) && !chess::applyMove(initial, true, move, after, error),
              "A king outside its palace was accepted");

        chess::Game game;
        check(game.board == initial && game.expected == initial && game.history.empty() && !game.pending,
              "Game default initialization changed");
        check(chess::Game_judge(game, {}) && game.result == chess::Result::BlackWon,
              "No legal move did not end the current side's game");
        check(chess::Game_reset(game, initial) && chess::Game_agreeDraw(game) && game.result == chess::Result::Draw,
              "Reset/draw state failed");
        check(chess::Game_reset(game, initial) && chess::Game_resign(game, false) &&
                  game.result == chess::Result::RedWon,
              "Resignation assigned the wrong winner");
        chess::Game_reset(game, initial);
        auto saved = game.board;
        check(!chess::Game_reset(game, invalid) && game.board == saved,
              "Invalid reset replaced the authoritative board");
        check(!chess::Game_judge(game, {"a3a4", "a3a4"}), "Duplicate legal list accepted");
        std::cout << "PASS FEN/move outputs, rule-data checks, game state, reset and outcomes\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
