#include "board_vision.hpp"
#include "app/config.hpp"
#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

BoardVision::BoardVision()
    : detector_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50))
{
    cv::aruco::DetectorParameters p;
    p.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    p.adaptiveThreshConstant = 5;
    detector_.setDetectorParameters(p);
}

void BoardVision::reset()
{
    filter_ = {};
    sameFrames_ = 0;
    located_ = false;
    rectified_.release();
}

bool BoardVision::locate(const cv::Mat& bgr, cv::Mat& rgb, std::string& error)
{
    error.clear();
    located_ = false;
    rgb.release();
    rectified_.release();
    if (bgr.empty() || bgr.type() != CV_8UC3) { reset(); error="没有有效彩色帧"; return false; }
    annotated_ = bgr.clone();
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector_.detectMarkers(bgr, corners, ids);
    cv::aruco::drawDetectedMarkers(annotated_, corners, ids);
    int index[4] = {-1,-1,-1,-1};
    bool duplicate = false;
    markersFound_ = 0;
    for (size_t i=0; i<ids.size(); ++i) {
        if (ids[i]<0 || ids[i]>3) continue;
        if (index[ids[i]]>=0) duplicate=true;
        else ++markersFound_;
        index[ids[i]]=int(i);
    }
    if (duplicate || markersFound_!=4) {
        reset(); error="需要四个清晰且不重复的角标ID 0-3"; return false;
    }

    // 板面与图像都归一化，避免EKF中像素与透视系数相差几个数量级。
    const double w=project::BoardWidth, h=project::BoardHeight;
    const float inset=float(project::MarkerInset), half=float(project::MarkerSide/2);
    const cv::Point2f centers[4]={{inset,inset},{float(w)-inset,inset},
                                {float(w)-inset,float(h)-inset},{inset,float(h)-inset}};
    const cv::Point2f offsets[4]={{-half,-half},{half,-half},{half,half},{-half,half}};
    std::vector<cv::Point2f> world, image;
    std::vector<scara::HomographyObservation> measurements;
    for (int id=0; id<4; ++id) {
        for (int corner=0; corner<4; ++corner) {
            const cv::Point2f a=centers[id]+offsets[corner], b=corners[index[id]][corner];
            world.push_back({float(a.x/w),float(a.y/h)});
            image.push_back({b.x/bgr.cols,b.y/bgr.rows});
            measurements.push_back({a.x/w,a.y/h,double(b.x)/bgr.cols,double(b.y)/bgr.rows});
        }
    }
    const std::vector<cv::Point2f> outline={image[0],image[5],image[10],image[15]};
    if (!cv::isContourConvex(outline) || cv::contourArea(outline,true)<=0) {
        reset(); error="角标排列不正确"; return false;
    }
    cv::Mat raw=cv::findHomography(world,image,0);
    if (raw.empty() || !cv::checkRange(raw)) { reset(); error="棋盘定位失败"; return false; }
    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(world,projected,raw);
    for (size_t i=0; i<world.size(); ++i) {
        const double dx=(projected[i].x-image[i].x)*bgr.cols;
        const double dy=(projected[i].y-image[i].y)*bgr.rows;
        if (!std::isfinite(dx+dy) || std::hypot(dx,dy)>5) {
            reset(); error="角标布局与中国象棋标定板不匹配"; return false;
        }
    }
    std::array<double,9> matrix{};
    std::copy(raw.ptr<double>(),raw.ptr<double>()+9,matrix.begin());
    const double now=cv::getTickCount()/cv::getTickFrequency();
    if (!filter_.initialized || now-lastTime_>2) {
        if (!scara::initializeHomography(filter_,matrix,0.001)) {
            reset(); error="EKF初始化失败"; return false;
        }
    } else {
        if (!scara::predictHomography(filter_,now-lastTime_,0.00001)) {
            reset(); error="EKF预测失败"; return false;
        }
        const double variance=std::pow(2.0/std::min(bgr.cols,bgr.rows),2);
        if (scara::updateHomography(filter_,measurements,variance)<12) {
            reset(); error="棋盘变化过大，正在重新定位"; return false;
        }
    }
    lastTime_=now;
    if (!scara::homographyMatrix(filter_,matrix)) {
        reset(); error="EKF映射无效"; return false;
    }
    cv::Mat H(3,3,CV_64F,matrix.data());
    // 验证真正使用的滤波结果，不能让旧映射在棋盘移动后继续驱动坐标。
    cv::perspectiveTransform(world,projected,H);
    for (size_t i=0;i<world.size();++i) {
        const double dx=(projected[i].x-image[i].x)*bgr.cols;
        const double dy=(projected[i].y-image[i].y)*bgr.rows;
        if (!std::isfinite(dx+dy) || std::hypot(dx,dy)>5) {
            reset(); error="滤波映射与当前帧不一致，正在重新定位"; return false;
        }
    }
    // H从归一化棋盘到归一化图像，显示和取点需要它的逆变换。
    const cv::Matx33d boardScale(2*w,0,0, 0,2*h,0, 0,0,1);
    const cv::Matx33d imageScale(1.0/bgr.cols,0,0, 0,1.0/bgr.rows,0, 0,0,1);
    const cv::Mat imageToBoard=cv::Mat(boardScale)*H.inv()*cv::Mat(imageScale);
    cv::warpPerspective(bgr,rectified_,imageToBoard,{int(2*w),int(2*h)});
    // YOLO只看棋子活动区域，四角标签不进入它的输入。
    const cv::Rect area(120,120,720,800);
    cv::Mat valid=cv::Mat::zeros(bgr.size(),CV_8UC1), warped;
    valid(cv::Rect(1,1,bgr.cols-2,bgr.rows-2)).setTo(255);
    cv::warpPerspective(valid,warped,imageToBoard,rectified_.size(),cv::INTER_NEAREST);
    if (cv::countNonZero(warped(area))!=area.area()) {
        reset(); error="棋子区域没有完整入镜"; return false;
    }
    cv::cvtColor(rectified_(area),rgb,cv::COLOR_BGR2RGB);
    located_=true;
    return true;
}

chess::Observation BoardVision::interpret(const std::vector<Detection>& pieces)
{
    chess::Observation result;
    for (auto& row:result.board) row.fill('.');
    if (!located_) { sameFrames_=0; return result; }
    result.complete=true;
    for (const auto& piece:pieces) {
        if (piece.classId<0 || piece.classId>=14 || !std::isfinite(piece.confidence)) {
            result.complete=false; continue;
        }
        const float x=(piece.x1+piece.x2)/2, y=(piece.y1+piece.y2)/2;
        if (!std::isfinite(x+y) || x<0 || x>720 || y<0 || y>800) {
            result.complete=false; continue;
        }
        const int col=int(std::lround((x-40)/80)), row=int(std::lround((y-40)/80));
        if (row<0 || row>=10 || col<0 || col>=9 ||
            std::abs(x-(40+80*col))>24 || std::abs(y-(40+80*row))>24) {
            result.complete=false; continue;
        }
        char& square=result.board[row][col];
        if (square!='.') { result.duplicate=true; square='?'; }
        else square=piece.confidence>=0.50f ? project::Classes[piece.classId] : '?';
        cv::rectangle(rectified_,{int(piece.x1)+120,int(piece.y1)+120},
                      {int(piece.x2)+120,int(piece.y2)+120},{0,180,0},2);
        cv::putText(rectified_,std::string(1,square),{int(x)+120,int(y)+120},
                    cv::FONT_HERSHEY_SIMPLEX,0.7,{0,80,240},2,cv::LINE_AA);
    }
    std::string reason;
    result.complete=result.complete && !result.duplicate && chess::validBoard(result.board,reason);
    if (result.complete && result.board==previous_) ++sameFrames_;
    else sameFrames_=result.complete ? 1 : 0;
    previous_=result.board;
    result.stable=result.complete && sameFrames_>=project::StableFrames;
    return result;
}
