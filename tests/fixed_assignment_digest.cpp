#include "geometry_fixture.hpp"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

int main()
{
    // 手写仿射关系独立生成输入，覆盖90交点、边界和识别异常
    auto input = fixture::input();
    double points[3][4] = {{100,600,10,60},{340,616,34,61.6},{145,330,14.5,33}};
    input.points = cv::Mat(3,4,CV_64F,points).clone();
    scara::Geometry original = fixture::geometry(input);
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
        for (double offset : {-.6, -.350001, -.349999, 0., .2, .349999, .350001, .6})
            for (int variation = 0; variation < 20; ++variation)
            {
                auto board = original;
                double rowOffset = variation % 2 == 0 ? offset / 3 : 0;
                double column = cell % 9 + offset, row = cell / 9 + rowOffset;
                cv::Point2d center(30 * column + 5 * row + 100, 2 * column - 30 * row + 600);
                scara::Detection piece;
                piece.classId = variation == 14 ? -1 : variation % 14;
                piece.confidence = variation % 3 == 0 ? .499999 : variation % 3 == 1 ? .5 : .9;
                if (variation >= 14) piece.confidence = .9;
                piece.bbox = {center.x - 10, center.y - 10, 20, 20};
                if (variation == 15) piece.confidence = std::numeric_limits<double>::quiet_NaN();
                if (variation == 16) piece.bbox.x = std::numeric_limits<double>::infinity();
                if (variation == 17) board.ready = false;
                if (variation == 18) piece.bbox.width = -1;
                std::vector<scara::Detection> pieces{piece};
                bool duplicate = variation == 10 || variation == 11;
                if (duplicate) pieces.push_back(piece);
                const std::string frame = variation == 19 ? "" : "frame";
                bool sceneClear = variation % 4 != 0;
                scara::Observation result;
                for (int repeat = 0; repeat < 3; ++repeat)
                    scara::Geometry_observe(board, frame.empty() ? frame : frame + std::to_string(repeat), pieces, sceneClear, result);
                // 独立预期：合格检测位于0.35格内才占格，重叠标成未知
                bool observed = (variation < 17 || variation == 18) && sceneClear;
                bool detected = variation < 14 && variation % 3 != 0;
                bool onCell = std::hypot(offset, rowOffset) <= .35;
                std::array<int, scara::Cells> expected;
                expected.fill(observed && sceneClear ? scara::Empty : scara::Unknown);
                if (observed && detected && onCell)
                    expected[cell] = duplicate ? scara::Unknown : piece.classId;
                bool valid = observed && sceneClear && detected && onCell && !duplicate;
                if (result.valid != valid || result.pieces != expected)
                {
                    std::cerr << "Assignment mismatch: cell=" << cell << " offset=" << offset
                              << " variation=" << variation << '\n';
                    return 1;
                }
                add(result.valid);
                for (int i = 0; i < scara::Cells; ++i)
                {
                    add(static_cast<uint64_t>(result.pieces[i]));
                }
                ++cases;
            }
    std::cout << "PASS " << cases << " analytic affine cases, digest " << std::hex << digest << '\n';
    return 0;
}
