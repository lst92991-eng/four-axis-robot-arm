#include "vision/geometry.hpp"
#include <cstdint>
#include <iostream>
#include <limits>

int main()
{
    scara::Board original;
#ifdef SCARA_OLD_GEOMETRY_API
    scara::Camera camera;
    camera.id = "assignment-equivalence-only";
    camera.imageSize = {1280, 720};
    scara::boardFromCorners(camera, "frame", {{{130, 650}, {1120, 630}, {1040, 80}, {170, 100}}}, true, original);
#else
    scara::boardFromCorners({{{130, 650}, {1120, 630}, {1040, 80}, {170, 100}}}, true, original);
#endif
    uint64_t digest = 1469598103934665603ULL;
    int cases = 0;
    auto add = [&](uint64_t value)
    {
        for (int byte = 0; byte < 8; ++byte)
        {
            digest = (digest ^ (value & 255)) * 1099511628211ULL;
            value >>= 8;
        }
    };
    for (int cell = 0; cell < scara::Cells; ++cell)
        for (double offset : {-.6, -.35, -.349999, 0., .349999, .35, .350001, .6})
            for (int variation = 0; variation < 20; ++variation)
            {
                auto board = original;
                double rowOffset = variation % 2 == 0 ? offset / 3 : 0;
                cv::Point2d center;
                scara::project(board.gridToPixel, {cell % 9 + offset, cell / 9 + rowOffset}, center);
                scara::Detection piece;
                piece.classId = variation == 14 ? -1 : variation % 14;
                piece.confidence = variation % 3 == 0 ? .499999 : variation % 3 == 1 ? .5 : .9;
                if (variation >= 14) piece.confidence = .9;
                piece.bbox = {center.x - 10, center.y - 10, 20, 20};
                if (variation == 15) piece.confidence = std::numeric_limits<double>::quiet_NaN();
                if (variation == 16) piece.bbox.x = std::numeric_limits<double>::infinity();
                if (variation == 17) board.orientationValid = false;
                if (variation == 18) piece.bbox.width = -1;
                std::vector<scara::Detection> pieces{piece};
                if (variation == 10 || variation == 11) pieces.push_back(piece);
                const std::string frame = variation == 19 ? "" : "frame";
                bool sceneClear = variation % 4 != 0;
                scara::Observation result;
#ifdef SCARA_OLD_GEOMETRY_API
                scara::ImageTransform transform;
                transform.sourceSize = camera.imageSize;
                scara::assignPieces(camera, board, frame, pieces, transform, scara::DetectionSpace::Original,
                                    sceneClear, {}, result);
#else
                scara::assignPieces(board, frame, pieces, sceneClear, result);
#endif
                add(result.valid);
                for (int i = 0; i < scara::Cells; ++i)
                {
                    add(static_cast<uint64_t>(result.pieces[i]));
                }
                ++cases;
            }
    std::cout << cases << " cases, digest " << std::hex << digest << '\n';
#ifdef SCARA_EXPECTED_ASSIGNMENT_DIGEST
    return digest == SCARA_EXPECTED_ASSIGNMENT_DIGEST ? 0 : 1;
#else
    return 0;
#endif
}
