#pragma once
#include <opencv2/core.hpp>
#include <array>
#include <string>
#include <vector>

struct Piece {
    char kind;
    cv::Point2d p;
    bool held = false;
};
struct CadInput {
    cv::Point2d world;
    int mouse = 0; // 1按下、2拖动、3释放，坐标是棋子顶面上的世界位置。
    bool overBoard = false, pause = false, step = false, quit = false;
};
void cadOpen(const cv::FileStorage &config, const cv::Mat &board,
             const std::array<cv::Mat, 14> &faces, bool visible);
cv::Mat cadCamera(const std::vector<Piece> &pieces, const std::array<double, 4> &joints);
cv::Mat cadScene(const std::vector<Piece> &pieces, const std::array<double, 4> &joints);
CadInput cadShow(const std::vector<Piece> &pieces, const std::array<double, 4> &joints,
                 const std::string &caption, const std::string &message, int dragging,
                 cv::Point2d ghost);
bool cadValidZ(double zMm);
void cadClose();
