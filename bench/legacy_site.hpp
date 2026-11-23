// 台架测量与离线回放使用的旧安装记录导入接口。
#pragma once
#include "app/game.hpp"

namespace scara
{
//---------台架安装数据接口区---------
// 从台架配置及其引用的记录中取得格点、机械参考、工作高度和对局参数。
void Bench_load(const std::string &path, GameConfig &config, Session &session);
// 从旧装配记录读取臂长、参考角、方向和传动比例。
bool Arm_load(const std::string &path, ArmGeometry &arm);
// 将旧 Z 示教记录换算为取子、净空、搬运和释放的 J1 角。
bool WorkHeights_load(const std::string &path, WorkHeights &heights);
// 按三个测点建系并应用逐格毫米修正，导出报告时可另取像素到基座的仿射矩阵。
void Bench_geometry(const cv::FileNode &config, Geometry &geometry, cv::Matx23d *pixelToBase = nullptr);
}
