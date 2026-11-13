#include "chess/session.hpp"
#include "vision/fixed_board.hpp"
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
int checks = 0;
void check(bool value, const char *message)
{
    ++checks;
    if (!value) throw std::runtime_error(message);
}

scara::Board grid()
{
    scara::Board result;
    scara::boardFromCorners({{{100, 600}, {900, 600}, {900, 60}, {100, 60}}}, true, result);
    return result;
}
scara::Observation observed(const chess::Board &board)
{
    scara::Observation o;
    o.frameId = "frame-1";
    o.valid = true;
    const std::string symbols = scara::PieceSymbols;
    for (int r = 0; r < 10; ++r)
        for (int c = 0; c < 9; ++c)
        {
            char piece = board[9 - r][c];
            o.pieces[r * 9 + c] = piece == '.' ? scara::Empty : int(symbols.find(piece));
        }
    return o;
}
scara::PlaneMap plane()
{
    std::vector<scara::PointPair> fit, verify;
    for (auto p : std::vector<cv::Point2d>{{20, 20}, {1220, 20}, {1220, 690}, {20, 690}, {300, 70}, {800, 650}})
        fit.push_back({p, p * .1});
    for (auto p : std::vector<cv::Point2d>{{200, 200}, {500, 350}, {800, 500}})
        verify.push_back({p, p * .1});
    scara::PlaneMap result;
    scara::calibratePlane("synthetic-only", 0, .001, fit, verify, result);
    return result;
}
} // namespace

int main()
{
    try
    {
        chess::Board initial;
        chess::initialBoard(initial);
        auto g = grid();
        auto o = observed(initial);
        chess::Observation c, checked;
        scara::chessObservation(o, c);
        check(c.valid && c.board == initial, "All 90 cells must preserve piece identity and reverse rows");
        auto unstable = o;
        unstable.valid = false;
        scara::chessObservation(unstable, checked);
        check(!checked.valid, "Unstable frames accepted");
        auto bad = o;
        bad.pieces[0] = scara::Unknown;
        scara::chessObservation(bad, checked);
        check(!checked.valid, "Unknown became empty");
        bad = o;
        bad.pieces[0] = 99;
        scara::chessObservation(bad, checked);
        check(!checked.valid, "Invalid class accepted");
        bad = o;
        bad.frameId.clear();
        scara::chessObservation(bad, checked);
        check(!checked.valid, "Missing frame identity accepted");
        bad = o;
        auto center = g.intersections[0];
        scara::Detection duplicate{12, .9, {center.x - 10, center.y - 10, 20, 20}};
        scara::assignPieces(g, "duplicate", {duplicate, duplicate}, true, bad);
        scara::chessObservation(bad, checked);
        check(!checked.valid, "Reported vision conflict accepted");
        auto unoriented = g;
        unoriented.orientationValid = false;
        scara::assignPieces(unoriented, "unconfirmed", {}, true, bad);
        scara::chessObservation(bad, checked);
        check(!checked.valid, "Unknown board orientation accepted");

        chess::Move human, robot;
        std::string error;
        chess::Board afterHuman;
        std::string moveText;
        check(chess::parseMove("a3a4", human) && chess::uci(human, moveText) && moveText == "a3a4",
              "UCI conversion failed");
        auto p = plane();
        scara::FixedBoard fixed;
        check(scara::FixedBoard_setup(p, g, fixed), "Fixed setup failed");
        cv::Point2d from, to, rejected;
        bool fromValid = scara::FixedBoard_target(fixed, human.fromCol, 9 - human.fromRow, from);
        bool toValid = scara::FixedBoard_target(fixed, human.toCol, 9 - human.toRow, to);
        check(fromValid && toValid, "UCI mapped to unavailable targets");
        check(cv::norm(from - cv::Point2d(10, 42)) < .001, "Source target XY incorrect");
        check(cv::norm(to - cv::Point2d(10, 36)) < .001, "Destination target XY incorrect");
        check(!scara::FixedBoard_setup(p, unoriented, fixed), "Unoriented target accepted");

        chess::Game game;
        const auto base = game.baseFen;
        check(chess::applyMove(initial, true, human, afterHuman, error), "Fixture human move failed");
        scara::chessObservation(observed(afterHuman), checked);
        check(chess::Game_acceptHuman(game, checked, {"a3a4"}, human), "Human move not accepted");
        check(game.history == "a3a4" && !game.redToMove && game.baseFen == base, "Base FEN/history/turn corrupted");
        chess::Observation beforeRobot, expected;
        scara::chessObservation(observed(afterHuman), beforeRobot);
        check(!chess::Game_prepareRobot(game, beforeRobot, "b9b8", {"a6a5"}, robot) && !game.pending,
              "Selected move outside the engine legal list was accepted");
        check(chess::Game_prepareRobot(game, beforeRobot, "a6a5", {"a6a5"}, robot), "Robot plan failed");
        check(game.pending && game.board == afterHuman && game.history == "a3a4", "Planned move committed too soon");
        scara::chessObservation(observed(game.expected), expected);
        check(!chess::Game_confirmRobot(game, expected, false) && game.pending && game.history == "a3a4",
              "Failed motion committed history");
        check(!chess::Game_confirmRobot(game, beforeRobot, true) && game.pending, "Wrong resulting board accepted");
        check(chess::Game_confirmRobot(game, expected, true) && !game.pending && game.history == "a3a4 a6a5",
              "Confirmed result not committed");
        std::cout << "PASS " << checks
                  << " game/geometry checks; legal lists are test fixtures, no engine or motor executed.\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
