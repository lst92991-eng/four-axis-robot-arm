#include "vision/geometry.hpp"
#include "vision/io.hpp"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <cmath>

namespace {
using namespace scara;
void require(bool v,const char* s) { if(!v) throw std::runtime_error(s); }
double number(const cv::FileNode& n) {
    require(n.isReal() || n.isInt(),"A real measured numeric value is required");
    double v=(double)n;require(std::isfinite(v),"Fill all measured values; NaN placeholders are not calibration");return v;
}
std::string relativePath(const std::string& config,const std::string& value) {
    require(!value.empty(),"Missing file path");auto p=std::filesystem::u8path(value);
    return (p.is_absolute()?p:std::filesystem::u8path(config).parent_path()/p).lexically_normal().u8string();
}
std::vector<PointPair> pairs(const cv::FileNode& node) {
    require(node.isSeq(),"fit_points/check_points must be sequences");std::vector<PointPair> out;
    for(const auto& n:node) {auto p=n["pixel"],q=n["base_xy"];require(p.size()==2 && q.size()==2,"Each pair needs pixel and base_xy");out.push_back({{number(p[0]),number(p[1])},{number(q[0]),number(q[1])}});}return out;
}
struct ArmTouch {
    cv::Point2d board;
    double j2Deg, j3MotorDeg;
};

std::vector<ArmTouch> armTouches(const cv::FileNode& node) {
    std::vector<ArmTouch> out;
    if (node.empty()) return out;
    require(node.isSeq(),"Arm touches must be a list");
    for (const auto& n : node) {
        const auto p=n["board_xy_mm"];
        require(p.isSeq() && p.size()==2,"Arm touch requires board_xy_mm");
        out.push_back({{number(p[0]),number(p[1])},number(n["j2_deg"]),number(n["j3_motor_deg"])});
    }
    return out;
}

void registerArm(const std::string& config,const std::string& output) {
    auto fs=readStorage(config);
    const auto fit=armTouches(fs["fit_touches"]), checks=armTouches(fs["check_touches"]);
    require(fit.size()>=3,"Record at least three distributed arm touches");
    const auto length=fs["arm_lengths_mm"];
    require(length.isSeq() && length.size()==2,"Two arm lengths are required");
    const double l1=number(length[0]),l2=number(length[1]),ratio=number(fs["j3_ratio"]);
    const double j2Scale=fs["j2_scale"].empty()?1:number(fs["j2_scale"]);
    require(l1>0 && l2>0 && ratio>0 && j2Scale>0,"Arm lengths and transmission ratios must be positive");
    const auto ref=fit.front();
    auto out=memoryStorage(true);
    out << "j2_reference_deg" << ref.j2Deg << "j3_reference_motor_deg" << ref.j3MotorDeg
        << "arm_lengths_mm" << "[" << l1 << l2 << "]" << "j2_scale" << j2Scale
        << "j3_ratio" << ratio << "solutions" << "[";
    int solutions=0;
    for (int s2 : {-1,1}) for (int s3 : {-1,1}) {
        auto phase=[&](const ArmTouch& p) {
            double q2=s2*j2Scale*(p.j2Deg-ref.j2Deg)*CV_PI/180;
            return cv::Vec2d(q2,q2+s3*(p.j3MotorDeg-ref.j3MotorDeg)*CV_PI/(180*ratio));
        };
        const int rows=2*static_cast<int>(fit.size());
        cv::Mat design(rows,6,CV_64F), observation(rows,1,CV_64F);
        for (int i=0;i<rows;i+=2) {
            const auto& p=fit[i/2];
            auto a=phase(p);
            double c1=std::cos(a[0]),s1=std::sin(a[0]),c2=std::cos(a[1]),s2a=std::sin(a[1]);
            const std::array<double,6> x{1,0,c1,-s1,c2,-s2a}, y{0,1,s1,c1,s2a,c2};
            std::copy(x.begin(),x.end(),design.ptr<double>(i));
            std::copy(y.begin(),y.end(),design.ptr<double>(i+1));
            observation.at<double>(i)=p.board.x;
            observation.at<double>(i+1)=p.board.y;
        }
        cv::Mat singular;
        cv::SVD::compute(design,singular);
        if (singular.at<double>(5)<=singular.at<double>(0)*1e-9) continue;
        cv::Mat linear;
        if (!cv::solve(design,observation,linear,cv::DECOMP_SVD)) continue;
        cv::Vec4d params(linear.at<double>(0),linear.at<double>(1),
                        std::atan2(linear.at<double>(3),linear.at<double>(2)),
                        std::atan2(linear.at<double>(5),linear.at<double>(4)));
        auto xy=[&](const ArmTouch& p,const cv::Vec4d& v) {
            const auto q=phase(p);
            double a=q[0]+v[2], b=q[1]+v[3];
            return cv::Point2d(v[0]+l1*std::cos(a)+l2*std::cos(b),v[1]+l1*std::sin(a)+l2*std::sin(b));
        };
        auto cost=[&](const cv::Vec4d& v) {
            double sum=0;
            for (const auto& p:fit) {
                auto e=xy(p,v)-p.board;
                sum+=e.dot(e);
            }
            return sum;
        };
        cv::Mat jac(rows,4,CV_64F), residual(rows,1,CV_64F);
        for (int iteration=0;iteration<20;++iteration) {
            for (int i=0;i<rows;i+=2) {
                const auto& p=fit[i/2];
                const auto q=phase(p);
                double a=q[0]+params[2], b=q[1]+params[3];
                const std::array<double,4> x{1,0,-l1*std::sin(a),-l2*std::sin(b)};
                const std::array<double,4> y{0,1,l1*std::cos(a),l2*std::cos(b)};
                std::copy(x.begin(),x.end(),jac.ptr<double>(i));
                std::copy(y.begin(),y.end(),jac.ptr<double>(i+1));
                const auto e=p.board-xy(p,params);
                residual.at<double>(i)=e.x;
                residual.at<double>(i+1)=e.y;
            }
            cv::Mat delta;
            if (!cv::solve(jac,residual,delta,cv::DECOMP_SVD)) break;
            cv::Vec4d step(delta.at<double>(0),delta.at<double>(1),delta.at<double>(2),delta.at<double>(3));
            auto next=params+step;
            const double old=cost(params);
            for (int i=0;i<6 && cost(next)>old;++i) {step*=.5;next=params+step;}
            if (cost(next)>=old) break;
            params=next;
        }
        ++solutions;
        out << "{" << "j2_sign" << s2 << "j3_sign" << s3
            << "j2_origin_board_mm" << "[" << params[0] << params[1] << "]"
            << "shoulder_ref_rad" << params[2] << "elbow_ref_rad" << std::remainder(params[3]-params[2],2*CV_PI)
            << "fit_rms_mm" << std::sqrt(cost(params)/fit.size()) << "fit_errors_mm" << "[";
        for (const auto& p:fit) out << cv::norm(xy(p,params)-p.board);
        out << "]" << "check_errors_mm" << "[";
        for (const auto& p:checks) out << cv::norm(xy(p,params)-p.board);
        out << "]" << "}";
    }
    require(solutions>0,"Arm touches do not span enough different poses");
    out << "]";
    writeBytes(output,out.releaseAndGetString());
}
cv::Mat imageFile(const std::string& path) {
    auto bytes=readBytes(path);std::vector<unsigned char> data(bytes.begin(),bytes.end());auto im=cv::imdecode(data,cv::IMREAD_COLOR);
    require(!im.empty(),"Cannot decode image");return im;
}
void saveImage(const std::string& path,const cv::Mat& image) {
    std::vector<unsigned char> b;require(cv::imencode(".png",image,b),"Cannot encode overlay");writeBytes(path,{b.begin(),b.end()});
}
Board getBoard(const cv::FileStorage& fs,const cv::Mat& image,const Camera& camera,const std::string& frame,const std::string& config) {
    Board board;std::string mode=(std::string)fs["mode"];
    if(mode=="manual") {
        auto n=fs["corners"];require(n.isSeq() && n.size()==4,"Four original-image corners required");std::array<cv::Point2d,4> corners;
        for(int i=0;i<4;++i) {require(n[i].size()==2,"Corner must have two coordinates");corners[i]={number(n[i][0]),number(n[i][1])};}
        board=boardFromCorners(camera,frame,corners,(int)fs["canonical_order_confirmed"]==1);
    } else {
        require(mode=="auto","mode must be auto or manual");board=locateBoard(image,camera,frame);
        std::string orientation=(std::string)fs["orientation"];
        if(board.geometryValid && (orientation=="manual_keep" || orientation=="manual_flip")) board=confirmOrientation(board,orientation=="manual_flip");
        else if(board.geometryValid && orientation=="reference") board=orientByReference(image,camera,board,loadOrientationReference(relativePath(config,(std::string)fs["orientation_reference"])));
        else require(orientation.empty() || orientation=="unconfirmed" || !board.geometryValid,"Unknown orientation mode");
    }
    return board;
}
Observation detectionsFile(const std::string& path,const Camera& camera,const Board& board,bool clear) {
    // Accept one JSON object or the last complete JSONL record from the current preview program.
    auto bytes=readBytes(path);
    if(std::filesystem::u8path(path).extension()==".jsonl") {std::istringstream s(bytes);std::string line,last;while(std::getline(s,line)) if(!line.empty()) last=line;bytes=last;}
    cv::FileStorage fs(bytes,cv::FileStorage::READ|cv::FileStorage::MEMORY);
    std::string frame=(std::string)fs["frame_id"],space=(std::string)fs["coordinates"];
    ImageTransform t;t.sourceSize=camera.imageSize;t.roi={0,0,camera.imageSize.width,camera.imageSize.height};
    auto source=fs["source_size"];if(source.isSeq() && source.size()==2) t.sourceSize={(int)source[0],(int)source[1]};
    auto roi=fs["roi"];if(roi.isSeq() && roi.size()==4) t.roi={(int)roi[0],(int)roi[1],(int)roi[2],(int)roi[3]};
    t.clockwiseDegrees=(int)fs["rotate_clockwise"];
    DetectionSpace kind=DetectionSpace::Original;
    if(space=="network") {kind=DetectionSpace::Network;auto s=fs["resized_size"],p=fs["padding"];require(s.size()==2 && p.size()==2,"Network detections need resized_size/padding");t.resizedSize={(int)s[0],(int)s[1]};t.padding={number(p[0]),number(p[1])};}
    else if(space=="roi" || (space.empty() && roi.isSeq())) kind=DetectionSpace::RotatedRoi;
    else require(space=="original" || space.empty(),"Unknown detection coordinate system");
    std::vector<Detection> ds;auto input=fs["detections"];require(input.isSeq(),"detections must be an array");
    for(const auto& d:input) {auto b=d["bbox"];require(b.size()==4,"bbox must be x1,y1,x2,y2");double x=number(b[0]),y=number(b[1]);ds.push_back({(int)d["class_id"],number(d["score"]),{x,y,number(b[2])-x,number(b[3])-y}});}
    return assignPieces(camera,board,frame,ds,t,kind,clear);
}
void inspect(const std::string& config,const std::string& output) {
    auto fs=readStorage(config);auto camera=readCamera(fs["camera"]);std::string frame=(std::string)fs["frame_id"];
    auto image=imageFile(relativePath(config,(std::string)fs["image"]));require(image.size()==camera.imageSize,"Image size differs from camera profile");
    auto board=getBoard(fs,image,camera,frame,config);std::optional<PlaneMap> plane;std::optional<Observation> observation;
    if(!fs["plane_file"].empty() && !((std::string)fs["plane_file"]).empty()) plane=loadPlane(relativePath(config,(std::string)fs["plane_file"]));
    if(!fs["detections_file"].empty() && !((std::string)fs["detections_file"]).empty()) observation=detectionsFile(relativePath(config,(std::string)fs["detections_file"]),camera,board,(int)fs["scene_clear"]==1);
    auto result=memoryStorage(true);result << "motion_enabled" << 0 << "single_frame_only" << 1 << "camera_id" << camera.id << "frame_id" << frame
        << "geometry_valid" << (int)board.geometryValid << "orientation_valid" << (int)board.orientationValid
        << "orientation_source" << board.orientationSource << "grid_score" << board.gridScore << "points" << "[";
    if(board.geometryValid) for(int r=0;r<Rows;++r) for(int c=0;c<Columns;++c) {
        auto p=board.intersections[r*Columns+c];result << "{" << "column" << c << "row" << r << "pixel_geometric" << "[" << p.x << p.y << "]";
        auto target=plane?gridTarget(*plane,board,c,r,frame):std::nullopt;result << "base_xy_valid" << (int)target.has_value();
        if(target) result << "base_xy_mm" << "[" << target->baseMm.x << target->baseMm.y << "]" << "plane_z_mm" << target->planeZmm;
        if(observation) result << "piece_id" << observation->pieces[r*Columns+c];result << "}";
    }
    result << "]";
    if(observation) result << "observation_valid" << (int)observation->valid << "issues" << observation->issues;
    writeBytes(output,result.releaseAndGetString());saveImage(output+".png",drawBoard(camera.geometricImage(image),board,observation?&*observation:nullptr));
    if(board.geometryValid) saveImage(output+".rectified.png",rectifiedBoard(camera.geometricImage(image),board));
    std::cout << "geometry=" << board.geometryValid << " orientation=" << board.orientationValid << " score=" << board.gridScore << "\nWrote " << output << " and overlay. No motion.\n";
}
}
int runGeometryTool(int argc,char** argv) {
    try {
        if(argc!=4) {std::cout << "Geometry only (C++17 / OpenCV " << CV_VERSION << ")\n"
            << "  scara_geometry_tool calibrate measured-pairs.yml plane-map.yml\n"
            << "  scara_geometry_tool inspect inspect.yml result.json\n"
            << "  scara_geometry_tool register-arm touches.json result.json\n"
            << "  scara_geometry_tool reference manual-inspect.yml orientation.yml\n";return argc==1?0:1;}
        const std::string command=argv[1],input=argv[2],output=argv[3];
        if(command=="calibrate") {
            auto fs=scara::readStorage(input);auto map=scara::calibratePlane(scara::readCamera(fs["camera"]),(std::string)fs["calibration_id"],number(fs["plane_z_mm"]),number(fs["allowed_error_mm"]),pairs(fs["fit_points"]),pairs(fs["check_points"]));
            std::cout << "fit RMS/max mm: " << map.fitError.rmsMm << "/" << map.fitError.maxMm << "\ncheck RMS/max mm: " << map.checkError.rmsMm << "/" << map.checkError.maxMm << '\n';
            for(size_t i=0;i<map.checkError.errorsMm.size();++i) std::cout << "check[" << i << "] error mm: " << map.checkError.errorsMm[i] << '\n';
            if(!map.accepted) {std::cerr << "REJECTED: errors exceed tolerance; no usable map saved.\n";return 2;}scara::savePlane(output,map);
        } else if(command=="inspect") inspect(input,output);
        else if(command=="register-arm") registerArm(input,output);
        else if(command=="reference") {
            auto fs=scara::readStorage(input);auto camera=scara::readCamera(fs["camera"]);auto image=imageFile(relativePath(input,(std::string)fs["image"]));
            auto board=getBoard(fs,image,camera,(std::string)fs["frame_id"],input);auto n=fs["reference_patch"];
            require(n.isSeq() && n.size()==4,"reference_patch [x,y,w,h] required in 432x480 rectified board");
            auto ref=scara::makeOrientationReference(image,camera,board,{(int)n[0],(int)n[1],(int)n[2],(int)n[3]});scara::saveOrientationReference(output,ref);
        } else throw std::runtime_error("Unknown command");
        return 0;
    } catch(const std::exception& e) {std::cerr << "ERROR: " << e.what() << '\n';return 1;}
}

#ifdef _WIN32
// Windows supplies UTF-16 arguments. Convert them explicitly; /utf-8 alone does not change argv.
int wmain(int argc,wchar_t** argv) {
    std::vector<std::string> utf8;utf8.reserve(argc);
    for(int i=0;i<argc;++i)utf8.push_back(std::filesystem::path(argv[i]).u8string());
    std::vector<char*> args;for(auto& value:utf8)args.push_back(value.data());
    return runGeometryTool(argc,args.data());
}
#else
int main(int argc,char** argv) { return runGeometryTool(argc,argv); }
#endif
