#include "vision/fixed_board.hpp"
#include "chess/session.hpp"
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
        std::vector<scara::PointPair> fit, checks;
        for (cv::Point2d pixel : std::vector<cv::Point2d>{{20, 20}, {1220, 20}, {1220, 690}, {20, 690}})
            fit.push_back({pixel, pixel * .1});
        checks.push_back({{600, 350}, {60, 35}});
        scara::PlaneMap plane;
        scara::calibratePlane("fixed-plane", 0, .001, fit, checks, plane);
        scara::Board grid;
        scara::boardFromCorners({{{100, 600}, {900, 600}, {900, 60}, {100, 60}}}, true, grid);
        scara::FixedBoard fixed;
        check(scara::FixedBoard_setup(plane, grid, fixed), "Cannot prepare fixed board");
        for (int row = 0; row < 10; ++row)
            for (int column = 0; column < 9; ++column)
            {
                cv::Point2d expected, actual;
                check(scara::gridTarget(plane, grid, column, row, expected), "Generic target missing");
                check(scara::FixedBoard_target(fixed, column, row, actual), "Cached target missing");
                check(actual == expected, "Cache changed a calibrated XY target");
            }

        chess::Board initial;
        chess::initialBoard(initial);
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
        scara::assignPieces(fixed.grid, "frame-1", pieces, true, observation);
        scara::chessObservation(observation, gameObservation);
        check(gameObservation.valid && gameObservation.board == initial, "Fixed layout changed piece identities");
        cv::Point2d target;
        check(scara::FixedBoard_target(fixed, 0, 3, target), "New frame cannot use fixed targets");
        auto sameTarget = target;
        scara::assignPieces(fixed.grid, "another-frame", pieces, true, observation);
        check(scara::FixedBoard_target(fixed, 0, 3, target) && target == sameTarget,
              "A new observation changed fixed coordinates");
        check(!scara::FixedBoard_target(fixed, 9, 3, target), "Out-of-range column accepted");

        scara::assignPieces(fixed.grid, "frame-2", pieces, false, observation);
        check(!observation.valid, "Occluded scene became a complete board");
        pieces.push_back(pieces.front());
        scara::assignPieces(fixed.grid, "frame-3", pieces, true, observation);
        check(!observation.valid, "Duplicate piece was silently accepted");

        auto unknown = grid;
        unknown.orientationValid = false;
        check(!scara::FixedBoard_setup(plane, unknown, fixed), "Unoriented setup accepted");
        check(!scara::FixedBoard_target(fixed, 0, 3, target), "Failed setup reused prior XY");
        check(scara::FixedBoard_setup(plane, grid, fixed), "Valid setup did not recover");
        check(scara::FixedBoard_setup(plane, fixed.grid, fixed), "In-place setup corrupted the grid input");
        scara::assignPieces(fixed.grid, "frame-4", {}, true, observation);
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
