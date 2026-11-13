#include "chess/session.hpp"
#include <iostream>
#include <stdexcept>

static void check(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}

static void observe(scara::Session &session, const chess::Board &board)
{
    static int frame = 0;
    std::vector<scara::Detection> pieces;
    std::string symbols = scara::PieceSymbols;
    for (int row = 0; row < 10; ++row)
        for (int col = 0; col < 9; ++col)
            if (board[9 - row][col] != '.')
            {
                auto center = session.layout.grid.intersections[row * 9 + col];
                pieces.push_back({int(symbols.find(board[9 - row][col])), .9, {center.x - 10, center.y - 10, 20, 20}});
            }
    for (int i = 0; i < 3; ++i)
        scara::Session_observe(session, std::to_string(++frame), pieces, true);
    check(session.observed.complete && session.observed.stable, "Fixture did not produce a stable board");
}

int main()
{
    try
    {
        scara::Session session;
        scara::Camera camera;
        camera.id = "session-test-only";
        camera.imageSize = {1280, 720};
        std::vector<scara::PointPair> fit, checks;
        auto base = [](cv::Point2d p) { return cv::Point2d(p.x * .25 - 150, -p.y * .3 - 150); };
        for (cv::Point2d p : std::vector<cv::Point2d>{{20, 20}, {1220, 20}, {1220, 690}, {20, 690}})
            fit.push_back({p, base(p)});
        checks.push_back({{600, 350}, base({600, 350})});
        scara::PlaneMap plane;
        scara::calibratePlane(camera, "session-plane", 0, .001, fit, checks, plane);
        scara::Board grid;
        scara::boardFromCorners(camera, "grid", {{{100, 600}, {900, 600}, {900, 60}, {100, 60}}}, true, grid);
        check(scara::FixedBoard_setup(plane, grid, session.layout), "Layout setup failed");
        check(scara::Arm_load("tests/fixtures/upper-app/arm-registration.json", session.arm), "Arm load failed");
        check(scara::WorkHeights_load("tests/fixtures/upper-app/pick-plan.json", session.heights),
              "Heights load failed");
        chess::Board initial;
        chess::initialBoard(initial);
        chess::Game_reset(session.game, initial, true, false);
        observe(session, initial);
        check(scara::Session_plan(session, "a3a4", {"a3a4"}) && session.steps.size() == 8 && session.game.pending,
              "Robot turn did not create a pending plan");
        check(session.game.board == initial && session.game.history.empty(), "Planning committed the turn");
        check(!scara::Session_confirm(session, false) && session.game.pending && session.steps.size() == 8,
              "Failed execution discarded its pending state");
        observe(session, session.game.expected);
        check(scara::Session_confirm(session, true) && session.game.history == "a3a4" && session.steps.empty(),
              "Verified placement did not commit exactly once");
        check(!scara::Session_confirm(session, true), "Same execution committed twice");
        chess::Move human;
        chess::Board afterHuman;
        std::string error;
        chess::parseMove("a6a5", human);
        chess::applyMove(session.game.board, false, human, afterHuman, error);
        observe(session, afterHuman);
        check(scara::Session_acceptHuman(session, {"a6a5"}, human) && session.game.history == "a3a4 a6a5",
              "Human response did not preserve full history");

        chess::Board capture;
        for (auto &row : capture)
            row.fill('.');
        capture[9][4] = 'K';
        capture[0][4] = 'k';
        capture[6][4] = 'P';
        capture[9][0] = 'R';
        capture[6][0] = 'p';
        chess::Game_reset(session.game, capture, true, false);
        observe(session, capture);
        check(!scara::Session_plan(session, "a0a3", {"a0a3"}) && !session.game.pending && session.steps.empty(),
              "Untaught capture zone still produced or locked a plan");
        session.capturePlaceTaught = true;
        session.capturePlaceMm = {-200, -250};
        check(scara::Session_plan(session, "a0a3", {"a0a3"}) && session.steps.size() == 16,
              "Capture did not plan victim removal before moving own piece");
        check(session.steps[6].action == scara::PickAction::ReleasePulse &&
                  session.steps[14].action == scara::PickAction::ReleasePulse,
              "Capture release order changed");
        observe(session, session.game.expected);
        check(scara::Session_confirm(session, true) && session.game.history == "a0a3", "Capture confirmation failed");
        std::cout
            << "PASS observation -> fixed XY -> IK/pick plan -> post-observation -> history; fixture moves, no I/O\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
