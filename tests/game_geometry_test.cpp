#include "chess/vision_bridge.hpp"
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
    scara::boardFromCorners("frame-1", {{{100, 600}, {900, 600}, {900, 60}, {100, 60}}}, true, result);
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
        scara::chessObservation(g, o, true, c);
        check(c.complete && c.stable && c.board == initial,
              "All 90 cells must preserve piece identity and reverse rows");
        scara::chessObservation(g, o, false, checked);
        check(!checked.stable, "Unstable frames accepted");
        auto bad = o;
        bad.pieces[0] = scara::Unknown;
        scara::chessObservation(g, bad, true, checked);
        check(!checked.complete, "Unknown became empty");
        bad = o;
        bad.pieces[0] = 99;
        scara::chessObservation(g, bad, true, checked);
        check(!checked.complete, "Invalid class accepted");
        bad = o;
        bad.frameId = "old-frame";
        scara::chessObservation(g, bad, true, checked);
        check(!checked.complete, "Mixed frames accepted");
        bad = o;
        bad.issues.push_back("duplicate cell");
        scara::chessObservation(g, bad, true, checked);
        check(!checked.complete, "Reported vision conflict accepted");
        auto unoriented = g;
        unoriented.orientationValid = false;
        scara::chessObservation(unoriented, o, true, checked);
        check(!checked.complete, "Unknown board orientation accepted");

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
        bool fromValid = scara::FixedBoard_target(fixed, human.fromCol, 9 - human.fromRow, "frame-1", from);
        bool toValid = scara::FixedBoard_target(fixed, human.toCol, 9 - human.toRow, "frame-1", to);
        check(fromValid && toValid, "UCI mapped to unavailable targets");
        check(cv::norm(from - cv::Point2d(10, 42)) < .001, "Source target XY incorrect");
        check(cv::norm(to - cv::Point2d(10, 36)) < .001, "Destination target XY incorrect");
        check(!scara::FixedBoard_target(fixed, 0, 3, "stale", rejected), "Stale target accepted");
        check(!scara::FixedBoard_setup(p, unoriented, fixed), "Unoriented target accepted");

        chess::Game game;
        const auto base = game.baseFen;
        check(chess::applyMove(initial, true, human, afterHuman, error), "Fixture human move failed");
        scara::chessObservation(g, observed(afterHuman), true, checked);
        check(chess::Game_acceptHuman(game, checked, {"a3a4"}, human), "Human move not accepted");
        check(game.history == "a3a4" && !game.redToMove && game.baseFen == base, "Base FEN/history/turn corrupted");
        chess::Observation beforeRobot, expected;
        scara::chessObservation(g, observed(afterHuman), true, beforeRobot);
        check(chess::Game_prepareRobot(game, beforeRobot, "a6a5", {"a6a5"}, robot), "Robot plan failed");
        check(game.pending && game.board == afterHuman && game.history == "a3a4", "Planned move committed too soon");
        scara::chessObservation(g, observed(game.expected), true, expected);
        check(!chess::Game_confirmRobot(game, expected, false) && game.pending && game.history == "a3a4",
              "Failed motion committed history");
        check(!chess::Game_confirmRobot(game, beforeRobot, true) && game.pending, "Wrong resulting board accepted");
        check(chess::Game_confirmRobot(game, expected, true) && !game.pending && game.history == "a3a4 a6a5",
              "Confirmed result not committed");
        chess::Board parsed;
        bool red = false;
        check(chess::parseFen(base, parsed, red, error) && red && parsed == initial, "Base FEN roundtrip failed");
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
