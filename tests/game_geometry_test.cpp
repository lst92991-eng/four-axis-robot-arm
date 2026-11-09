#include "chess/vision_bridge.hpp"
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
int checks = 0;
void check(bool value, const char* message) {
    ++checks; if (!value) throw std::runtime_error(message);
}
scara::Camera camera() {
    scara::Camera c; c.id = "synthetic-game-test-only"; c.imageSize = {1280,720}; return c;
}
scara::Board grid() {
    return scara::boardFromCorners(camera(), "frame-1", {{{100,600},{900,600},{900,60},{100,60}}}, true);
}
scara::Observation observed(const chess::Board& board) {
    scara::Observation o; o.cameraId = camera().id; o.frameId = "frame-1"; o.valid = true;
    const std::string symbols = scara::PieceSymbols;
    for (int r = 0; r < 10; ++r) for (int c = 0; c < 9; ++c) {
        char piece = board[9-r][c];
        o.pieces[r*9+c] = piece == '.' ? scara::Empty : int(symbols.find(piece));
    }
    return o;
}
scara::PlaneMap plane() {
    std::vector<scara::PointPair> fit, verify;
    for (auto p : std::vector<cv::Point2d>{{20,20},{1220,20},{1220,690},{20,690},{300,70},{800,650}})
        fit.push_back({p, p * .1});
    for (auto p : std::vector<cv::Point2d>{{200,200},{500,350},{800,500}})
        verify.push_back({p, p * .1});
    return scara::calibratePlane(camera(), "synthetic-only", 0, .001, fit, verify);
}
}

int main() {
    try {
        const auto initial = chess::initialBoard(); auto g = grid(); auto o = observed(initial);
        auto c = scara::chessObservation(g,o,true);
        check(c.complete && c.stable && c.board == initial, "All 90 cells must preserve piece identity and reverse rows");
        check(!scara::chessObservation(g,o,false).stable, "Unstable frames accepted");
        auto bad = o; bad.pieces[0] = scara::Unknown;
        check(!scara::chessObservation(g,bad,true).complete, "Unknown became empty");
        bad=o; bad.pieces[0]=99;
        check(!scara::chessObservation(g,bad,true).complete, "Invalid class accepted");
        bad=o; bad.frameId="old-frame";
        check(!scara::chessObservation(g,bad,true).complete, "Mixed frames accepted");
        bad=o; bad.issues.push_back("duplicate cell");
        check(!scara::chessObservation(g,bad,true).complete, "Reported vision conflict accepted");
        auto unoriented=g; unoriented.orientationValid=false;
        check(!scara::chessObservation(unoriented,o,true).complete, "Unknown board orientation accepted");

        chess::Move human, robot; std::string error; chess::Board afterHuman;
        check(chess::parseMove("a3a4",human) && chess::uci(human)=="a3a4", "UCI conversion failed");
        auto p=plane(); auto xy=scara::chessMoveTargets(p,g,human,"frame-1");
        check(xy && xy->from.column==0 && xy->from.row==3 && xy->to.row==4, "UCI mapped to wrong grid rank");
        check(cv::norm(xy->from.baseMm-cv::Point2d(10,42))<.001, "Source target XY incorrect");
        check(cv::norm(xy->to.baseMm-cv::Point2d(10,36))<.001, "Destination target XY incorrect");
        check(!scara::chessMoveTargets(p,g,human,"stale"), "Stale target accepted");
        check(!scara::chessMoveTargets(p,unoriented,human,"frame-1"), "Unoriented target accepted");

        chess::Game game; const auto base=game.baseFen();
        check(chess::applyMove(initial,true,human,afterHuman,error), "Fixture human move failed");
        check(game.acceptHuman(scara::chessObservation(g,observed(afterHuman),true),{"a3a4"},human), "Human move not accepted");
        check(game.history()=="a3a4" && !game.redToMove() && game.baseFen()==base, "Base FEN/history/turn corrupted");
        auto beforeRobot=scara::chessObservation(g,observed(afterHuman),true);
        check(game.prepareRobot(beforeRobot,"a6a5",{"a6a5"},robot), "Robot plan failed");
        check(game.pending() && game.board()==afterHuman && game.history()=="a3a4", "Planned move committed too soon");
        auto expected=scara::chessObservation(g,observed(game.expectedBoard()),true);
        check(!game.confirmRobot(expected,false) && game.pending() && game.history()=="a3a4", "Failed motion committed history");
        check(!game.confirmRobot(beforeRobot,true) && game.pending(), "Wrong resulting board accepted");
        check(game.confirmRobot(expected,true) && !game.pending() && game.history()=="a3a4 a6a5", "Confirmed result not committed");
        chess::Board parsed; bool red=false;
        check(chess::parseFen(base,parsed,red,error) && red && parsed==initial, "Base FEN roundtrip failed");
        std::cout << "PASS " << checks << " game/geometry checks; legal lists are test fixtures, no engine or motor executed.\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
