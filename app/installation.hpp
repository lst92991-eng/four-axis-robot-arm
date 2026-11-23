// 固定前的实测机位，供离线阅读。15-site 用新实测值生成同格式的本机版本。
#pragma once
#include "app/game.hpp"

namespace scara
{
//---------安装参数区---------
inline void Setup_game(GameConfig &config, Session &session)
{
    // (9,0)、(9,8)、(0,0)，每点依次写原图像素和基座毫米。
    BoardPoint redLeft = {{447.09289980014006, 55.57567622139173}, {-173.8023129975593, -387.5925717144836}};
    BoardPoint redRight = {{408.1932172990374, 553.9950271059687}, {65.78688259030096, -424.91897760487274}};
    BoardPoint blackLeft = {{982.6774964405543, 96.55278255369922}, {-134.82560508806856, -133.70063738059653}};
    Geometry_calibrate(redLeft, redRight, blackLeft, session.layout);
    // 这一机位的红左角有单格修正，只改机械臂到达的毫米位置。
    session.layout.cell[9][0].xy += cv::Point2d(1.036, 3.044);

    // 关节参考角和 J3 传动方向取自这台机械臂。
    session.arm.upperMm = 275;
    session.arm.forearmMm = 275;
    session.arm.j3Ratio = 3;
    session.arm.j3Sign = -1;
    session.arm.j2ReferenceDeg = -53.23;
    session.arm.j3ReferenceMotorDeg = -76.971;
    session.arm.j2Scale = 0.8;
    session.arm.shoulderReferenceRad = -0.812579750641072;
    session.arm.elbowReferenceRad = -1.513770027184262;

    // J1 吸取、搬运和释放位置，单位为电机度。
    session.heights.pickupDeg = -20225.7605;
    session.heights.travelDeg = -18219.66;
    session.heights.releaseDeg = -20225.7605;
    // 抬到搬运层后开始横移。
    session.heights.clearanceDeg = session.heights.travelDeg;
    // 观察位用 J2 和 J3 的驱动角，盘外弃子点用基座 XY 毫米。
    session.park = {0.404, -23.939};
    session.capturePlaceMm = {165.15406706485533, -245.16524414593607};

    // 相机地址、原图识别区域和引擎目录。
    config.cameraUrl = "rtsp://192.168.31.123/live/0";
    config.visionRoi = {370, 28, 650, 600};
    config.engineDir = "engine/pikafish-2023-03-05";

    // 按 J1、J2、J3 排列，速度用度/秒，加速度用度/秒²。
    config.motion.velocity = {4800, 42, 106};
    config.motion.acceleration = {5700, 110, 180};
}
}
