#include "vision/fixed_board.hpp"
#include "chess/vision_bridge.hpp"
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
        scara::Camera camera;
        camera.id = "fixed-board-test-only";
        camera.imageSize = {1280, 720};
        std::vector<scara::PointPair> fit, checks;
        for (cv::Point2d pixel : std::vector<cv::Point2d>{{20, 20}, {1220, 20}, {1220, 690}, {20, 690}})
            fit.push_back({pixel, pixel * .1});
        checks.push_back({{600, 350}, {60, 35}});
        scara::PlaneMap plane;
        scara::calibratePlane(camera, "fixed-plane", 0, .001, fit, checks, plane);
        scara::Board grid;
        scara::boardFromCorners(camera, "grid-0", {{{100, 600}, {900, 600}, {900, 60}, {100, 60}}}, true, grid);
        scara::FixedBoard fixed;
        check(scara::FixedBoard_setup(plane, grid, fixed), "Cannot prepare fixed board");
        for (int row = 0; row < 10; ++row)
            for (int column = 0; column < 9; ++column)
            {
                scara::TargetXY expected, actual;
                check(scara::gridTarget(plane, grid, column, row, "grid-0", expected), "Generic target missing");
                check(scara::FixedBoard_target(fixed, column, row, "grid-0", actual), "Cached target missing");
                check(actual.baseMm == expected.baseMm && actual.calibrationId == expected.calibrationId,
                      "Cache changed a calibrated XY target");
            }

        chess::Board initial = chess::initialBoard();
        std::vector<scara::Detection> pieces;
        std::string symbols = scara::PieceSymbols;
        for (int row = 0; row < 10; ++row)
            for (int column = 0; column < 9; ++column)
                if (initial[9 - row][column] != '.')
                {
                    cv::Point2d center = grid.intersections[row * 9 + column];
                    pieces.push_back(
                        {int(symbols.find(initial[9 - row][column])), .9, {center.x - 10, center.y - 10, 20, 20}});
                }
        scara::Observation observation;
        chess::Observation gameObservation;
        scara::FixedBoard_assign(fixed, "frame-1", pieces, true, observation);
        scara::chessObservation(fixed.grid, observation, true, gameObservation);
        check(gameObservation.complete && gameObservation.board == initial, "Fixed layout changed piece identities");
        scara::TargetXY target;
        check(scara::FixedBoard_target(fixed, 0, 3, "frame-1", target), "New frame cannot use fixed targets");
        check(!scara::FixedBoard_target(fixed, 0, 3, "grid-0", target) && target.frameId.empty(),
              "Stale frame retained a target");
        check(!scara::FixedBoard_target(fixed, 9, 3, "frame-1", target), "Out-of-range column accepted");

        scara::FixedBoard_assign(fixed, "frame-2", pieces, false, observation);
        check(!observation.valid, "Occluded scene became a complete board");
        pieces.push_back(pieces.front());
        scara::FixedBoard_assign(fixed, "frame-3", pieces, true, observation);
        check(!observation.valid, "Duplicate piece was silently accepted");

        auto unknown = grid;
        unknown.orientationValid = false;
        check(!scara::FixedBoard_setup(plane, unknown, fixed), "Unoriented setup accepted");
        check(!scara::FixedBoard_target(fixed, 0, 3, "grid-0", target), "Failed setup reused prior XY");
        check(scara::FixedBoard_setup(plane, grid, fixed), "Valid setup did not recover");
        check(scara::FixedBoard_setup(plane, fixed.grid, fixed), "In-place setup corrupted the grid input");
        scara::FixedBoard_assign(fixed, "frame-4", {}, true, observation);
        check(observation.valid && observation.pieces[0] == scara::Empty, "Empty observation retained old pieces");
        std::cout << "PASS all 90 cached targets, piece mapping, new/stale frames, failed setup and recovery\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
