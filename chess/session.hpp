#pragma once
#include "board.hpp"
#include "vision/fixed_board.hpp"
#include "motion/plan.hpp"

namespace scara
{
// 一个对局保存一份识别结果和一份待执行计划。引擎查询与实际IO由调用方负责。
struct Session
{
    chess::Game game;
    FixedBoard layout;
    Stabilizer stable;
    ArmGeometry arm;
    WorkHeights heights;
    chess::Observation observed;
    std::vector<PickStep> steps;
    bool capturePlaceTaught = false;
    cv::Point2d capturePlaceMm;
};

// 将稳定的格点身份转换为黑方起算的9x10棋局数组。
void chessObservation(const Observation &source, chess::Observation &result);
void Session_observe(Session &session, const std::string &frameId, const std::vector<Detection> &pieces,
                     bool sceneClear);
bool Session_acceptHuman(Session &session, const std::vector<std::string> &legal, chess::Move &move);
bool Session_plan(Session &session, const std::string &choice, const std::vector<std::string> &legal);
bool Session_confirm(Session &session, bool armSucceeded);
} // namespace scara
