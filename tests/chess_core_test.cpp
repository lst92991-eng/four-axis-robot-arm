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
        chess::Board initial = chess::initialBoard(), after;
        check(!chess::validSquares({0, 9, 1, 0}) && !chess::validSquares({-1, 0, 1, 0}),
              "Invalid coordinates could index the fixed board");
        chess::Move move;

        check(chess::uci(move).empty(), "Invalid move retained output text");
        check(chess::parseMove("a3a4", move) && chess::uci(move) == "a3a4", "Move conversion failed");
        check(chess::validTransfer(initial, true, move), "Selected pawn could not move");
        after = initial;
        chess::movePiece(after, move);
        check(after[6][0] == '.' && after[5][0] == 'P',
              "Move changed the wrong squares");
        // 只接受起点、终点和实际棋子都吻合的唯一合法走法。
        const chess::Move pawnMove{6, 0, 5, 0};
        const chess::MoveList legal{pawnMove, {6, 2, 5, 2}};
        chess::Board seen = after;
        check(chess::inferMove(initial, seen, legal, move) && chess::sameMove(move, pawnMove),
              "Board difference did not identify the moved piece");
        seen[6][2] = '.';
        seen[5][2] = 'P';
        check(!chess::inferMove(initial, seen, legal, move),
              "Two moved pieces became one move");
        seen = after;
        seen[5][0] = 'C';
        check(!chess::inferMove(initial, seen, legal, move),
              "Changed piece identity accepted");
        seen = after;
        check(!chess::inferMove(initial, seen, {pawnMove, pawnMove}, move),
              "Repeated matching move accepted");
        check(!chess::inferMove(initial, seen, {{6, 2, 5, 2}}, move),
              "Observed move absent from the legal list was accepted");
        seen = initial;
        check(!chess::inferMove(initial, seen, legal, move) &&
                  !chess::validSquares(move),
              "Unchanged board did not clear the output move");
        chess::Board beforeCapture = initial;
        beforeCapture[3][0] = '.';
        beforeCapture[5][0] = 'p';
        seen = beforeCapture;
        seen[5][0] = 'P';
        seen[6][0] = '.';
        check(chess::inferMove(beforeCapture, seen, legal, move) && chess::sameMove(move, pawnMove),
              "The landing piece was not matched when it covered a captured piece");
        chess::Board invalid = initial;
        invalid[0][0] = '?';
        check(!chess::validBoard(invalid), "Invalid board was accepted at the input boundary");
        check(!chess::parseMove("a3a3", move) && !chess::parseMove("j3a4", move), "Invalid UCI coordinates accepted");
        check(chess::parseMove("e0e3", move) && !chess::validTransfer(initial, true, move),
              "A king outside its palace was accepted");

        chess::Game game;
        check(game.board == initial && game.history.empty(),
              "Game default initialization changed");
        const std::string initialFen = "rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR";
        check(game.baseFen == initialFen + " w - - 0 1", "Red opening FEN changed");
        chess::Game_judge(game, {});
        check(game.result == chess::Result::BlackWon,
              "No legal move did not end the current side's game");
        check(chess::Game_reset(game, initial, false) && game.baseFen == initialFen + " b - - 0 1",
              "Black opening FEN changed");
        auto saved = game.board;
        check(!chess::Game_reset(game, invalid) && game.board == saved,
              "Invalid reset replaced the authoritative board");
        check(chess::Game_reset(game,after) && chess::Game_reset(game,game.board) && game.board==after &&
                  game.positions.size()==1 && game.history.empty(),"In-place reset lost the confirmed board");
        game.positions = {after, initial, initial, after, after};
        chess::Game_judge(game, {{6,2,5,2}});
        check(game.result == chess::Result::Playing,
              "Equal boards with the other side to move counted toward repetition");
        game.positions[2] = after;
        chess::Game_judge(game, {{6,2,5,2}});
        check(game.result == chess::Result::ReviewRequired,
              "A third equal board with the same side to move was missed");
        std::cout << "PASS FEN/move outputs, rule-data checks, game state, reset and outcomes\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
