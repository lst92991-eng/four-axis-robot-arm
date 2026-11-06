#include "geometry.hpp"
#include "io.hpp"
#include <opencv2/geometry.hpp>
#include <opencv2/calib.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace scara {
namespace {
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
bool finite(cv::Point2d p) { return std::isfinite(p.x) && std::isfinite(p.y); }
cv::Matx33d matrix33(const cv::Mat& input) {
    require(input.rows == 3 && input.cols == 3 && input.channels()==1, "Expected a scalar 3x3 matrix");
    cv::Mat m; input.convertTo(m,CV_64F); require(cv::checkRange(m),"Nonfinite matrix");
    cv::Matx33d h; std::copy(m.ptr<double>(),m.ptr<double>()+9,h.val);
    require(std::abs(cv::determinant(h)) > 1e-14,"Singular matrix"); return h;
}
std::vector<cv::Point2d> hull(const std::vector<cv::Point2d>& points) {
    std::vector<cv::Point2f> p(points.begin(),points.end()), h; cv::convexHull(p,h);
    require(h.size() >= 3 && std::abs(cv::contourArea(h)) > 1.0,"Points are collinear or too concentrated");
    return {h.begin(),h.end()};
}
bool contains(const std::vector<cv::Point2d>& h, cv::Point2d p) {
    std::vector<cv::Point2f> f(h.begin(),h.end());
    return f.size() >= 3 && cv::pointPolygonTest(f,p,true) >= -1e-4;
}
ErrorReport errorReport(const Camera& camera, const cv::Matx33d& h, const std::vector<PointPair>& pairs) {
    ErrorReport r;
    for (const auto& p:pairs) { double e=cv::norm(project(h,camera.geometricPixel(p.pixel))-p.baseXY);
        r.errorsMm.push_back(e); r.maxMm=std::max(r.maxMm,e); r.rmsMm+=e*e; }
    r.rmsMm=std::sqrt(r.rmsMm/pairs.size()); return r;
}
void writePairs(cv::FileStorage& fs,const char* key,const std::vector<PointPair>& pairs) {
    fs << key << "["; for(const auto& p:pairs) fs << "{" << "pixel" << "[" << p.pixel.x << p.pixel.y << "]"
        << "base_xy" << "[" << p.baseXY.x << p.baseXY.y << "]" << "}"; fs << "]";
}
std::vector<PointPair> readPairs(const cv::FileNode& node) {
    require(node.isSeq(),"Missing point pair sequence"); std::vector<PointPair> pairs;
    for(const auto& n:node) { auto a=n["pixel"],b=n["base_xy"];
        require(a.isSeq() && b.isSeq() && a.size()==2 && b.size()==2,"Invalid point pair");
        for(auto v:{a[0],a[1],b[0],b[1]}) require(v.isReal() || v.isInt(),"Point coordinates must be numeric, not null/placeholders");
        pairs.push_back({{(double)a[0],(double)a[1]},{(double)b[0],(double)b[1]}}); }
    return pairs;
}
}

cv::Point2d project(const cv::Matx33d& h,cv::Point2d p) {
    require(finite(p),"Nonfinite point"); const auto q=h*cv::Vec3d(p.x,p.y,1);
    require(std::isfinite(q[2]) && std::abs(q[2])>1e-10,"Projection at infinity");
    cv::Point2d result(q[0]/q[2],q[1]/q[2]); require(finite(result),"Invalid projection"); return result;
}
void Camera::validate() const {
    require(!id.empty() && imageSize.width>0 && imageSize.height>0,"Camera ID/size required");
    require(K.empty()==distortion.empty(),"Supply both intrinsics and distortion, or neither");
    if (!K.empty()) { auto k=matrix33(K); require(k(0,0)>0 && k(1,1)>0,"Invalid focal length");
        require(cv::checkRange(distortion),"Nonfinite distortion");
        const auto n=distortion.total(); require(n==4 || n==5 || n==8 || n==12 || n==14,"Unsupported distortion vector"); }
}
bool Camera::sameGeometry(const Camera& other) const {
    if(id!=other.id || imageSize!=other.imageSize || K.empty()!=other.K.empty() || distortion.total()!=other.distortion.total()) return false;
    if(K.empty()) return true;
    cv::Mat a,b,da,db;K.convertTo(a,CV_64F);other.K.convertTo(b,CV_64F);
    distortion.reshape(1,1).convertTo(da,CV_64F);other.distortion.reshape(1,1).convertTo(db,CV_64F);
    return cv::norm(a,b,cv::NORM_INF)<1e-12 && cv::norm(da,db,cv::NORM_INF)<1e-12;
}
cv::Point2d Camera::geometricPixel(cv::Point2d raw) const {
    validate(); require(finite(raw),"Nonfinite pixel"); if(K.empty()) return raw;
    std::vector<cv::Point2d> in{raw},out; cv::undistortPoints(in,out,K,distortion,cv::noArray(),K); return out[0];
}
cv::Mat Camera::geometricImage(const cv::Mat& raw) const {
    validate(); require(!raw.empty() && raw.size()==imageSize,"Camera image size changed");
    require(raw.depth()==CV_8U && (raw.channels()==1 || raw.channels()==3),"Expected 8-bit gray or BGR source image");
    if(K.empty()) return raw.clone(); cv::Mat result; cv::undistort(raw,result,K,distortion,K); return result;
}
PlaneMap calibratePlane(const Camera& camera,const std::string& id,double z,double tolerance,
                       const std::vector<PointPair>& fit,const std::vector<PointPair>& checks) {
    camera.validate(); require(!id.empty(),"Calibration ID required");
    require(std::isfinite(z) && std::isfinite(tolerance) && tolerance>0,"Real plane Z and error tolerance required");
    require(fit.size()>=4 && checks.size()>=3,"Need >=4 fit pairs and >=3 independent check pairs");
    std::vector<cv::Point2d> pixels,xy;
    auto checkPair=[&](const PointPair& p) {
        require(finite(p.pixel) && finite(p.baseXY),"Invalid calibration pair");
        require(p.pixel.x>=0 && p.pixel.y>=0 && p.pixel.x<camera.imageSize.width && p.pixel.y<camera.imageSize.height,"Calibration pixel outside original frame");
    };
    for(const auto& p:fit) { checkPair(p); pixels.push_back(camera.geometricPixel(p.pixel)); xy.push_back(p.baseXY); }
    auto coverage=hull(pixels); hull(xy);
    for(size_t i=0;i<fit.size();++i) for(size_t j=0;j<i;++j)
        require(cv::norm(fit[i].pixel-fit[j].pixel)>1 && cv::norm(fit[i].baseXY-fit[j].baseXY)>1e-6,"Duplicate fit points");
    for(size_t i=0;i<checks.size();++i) { checkPair(checks[i]);
        for(const auto& p:fit) require(cv::norm(p.pixel-checks[i].pixel)>1 && cv::norm(p.baseXY-checks[i].baseXY)>1e-6,"Check point reused from fit set");
        for(size_t j=0;j<i;++j) require(cv::norm(checks[i].pixel-checks[j].pixel)>1,"Duplicate check points"); }
    PlaneMap m; m.camera=camera; m.id=id; m.planeZmm=z; m.allowedErrorMm=tolerance;
    m.pixelToBase=matrix33(cv::findHomography(pixels,xy,0)); m.validPixelHull=coverage;
    m.fitPoints=fit; m.checkPoints=checks; m.fitError=errorReport(camera,m.pixelToBase,fit);
    m.checkError=errorReport(camera,m.pixelToBase,checks);
    m.accepted=m.fitError.maxMm<=tolerance && m.checkError.maxMm<=tolerance; return m;
}
void saveCamera(cv::FileStorage& fs,const Camera& c) {
    fs << "camera" << "{" << "id" << c.id << "width" << c.imageSize.width << "height" << c.imageSize.height
       << "K" << c.K << "distortion" << c.distortion << "}";
}
Camera readCamera(const cv::FileNode& n) {
    Camera c; n["id"]>>c.id; n["width"]>>c.imageSize.width; n["height"]>>c.imageSize.height;
    if(!n["K"].empty()) n["K"]>>c.K; if(!n["distortion"].empty()) n["distortion"]>>c.distortion;
    c.validate(); return c;
}
void savePlane(const std::string& path,const PlaneMap& m) {
    require(m.accepted,"Calibration failed; not saving a usable plane map");
    auto fs=memoryStorage();
    fs << "schema_version" << 1 << "calibration_id" << m.id << "plane_z_mm" << m.planeZmm
       << "allowed_error_mm" << m.allowedErrorMm; saveCamera(fs,m.camera);
    fs << "pixel_to_base" << cv::Mat(m.pixelToBase) << "fit_rms_mm" << m.fitError.rmsMm
       << "fit_max_mm" << m.fitError.maxMm << "check_rms_mm" << m.checkError.rmsMm << "check_max_mm" << m.checkError.maxMm;
    fs << "fit_errors_mm" << m.fitError.errorsMm << "check_errors_mm" << m.checkError.errorsMm;
    writePairs(fs,"fit_points",m.fitPoints); writePairs(fs,"check_points",m.checkPoints);
    writeBytes(path,fs.releaseAndGetString());
}
PlaneMap loadPlane(const std::string& path) {
    auto fs=readStorage(path);
    require((int)fs["schema_version"]==1,"Unsupported calibration schema");
    require(fs["plane_z_mm"].isReal() || fs["plane_z_mm"].isInt(),"Plane height is missing");
    require(fs["allowed_error_mm"].isReal() || fs["allowed_error_mm"].isInt(),"Validation tolerance is missing");
    PlaneMap m=calibratePlane(readCamera(fs["camera"]),(std::string)fs["calibration_id"],(double)fs["plane_z_mm"],
        (double)fs["allowed_error_mm"],readPairs(fs["fit_points"]),readPairs(fs["check_points"]));
    cv::Mat saved; fs["pixel_to_base"]>>saved; auto h=matrix33(saved);
    for(const auto& p:m.fitPoints) require(cv::norm(project(h,m.camera.geometricPixel(p.pixel))-project(m.pixelToBase,m.camera.geometricPixel(p.pixel)))<1e-5,"Stored map differs from calibration samples");
    require(m.accepted,"Independent validation failed"); return m;
}

cv::Point2d ImageTransform::toOriginal(cv::Point2d p,DetectionSpace space) const {
    require(sourceSize.width>0 && sourceSize.height>0 && finite(p),"Invalid source geometry");
    if(space!=DetectionSpace::Original) {
        require(roi.width>0 && roi.height>0 && roi.x>=0 && roi.y>=0 && roi.x+roi.width<=sourceSize.width && roi.y+roi.height<=sourceSize.height,"Invalid ROI");
        require(clockwiseDegrees==0 || clockwiseDegrees==90 || clockwiseDegrees==180 || clockwiseDegrees==270,"Unsupported ROI rotation");
        const bool swap=clockwiseDegrees==90 || clockwiseDegrees==270;
        const cv::Size rotated=swap?cv::Size(roi.height,roi.width):roi.size();
        if(space==DetectionSpace::Network) {
            require(resizedSize.width>0 && resizedSize.height>0 && finite(padding),"Actual resized size and letterbox padding required");
            p.x=(p.x-padding.x)*rotated.width/resizedSize.width;
            p.y=(p.y-padding.y)*rotated.height/resizedSize.height;
        }
        require(p.x>=0 && p.y>=0 && p.x<rotated.width && p.y<rotated.height,"Detection outside unpadded ROI");
        switch(clockwiseDegrees) {
        case 90:p={p.y,roi.height-1-p.x};break;
        case 180:p={roi.width-1-p.x,roi.height-1-p.y};break;
        case 270:p={roi.width-1-p.y,p.x};break;
        default:break;
        }
        p+=cv::Point2d(roi.x,roi.y);
    }
    require(p.x>=0 && p.y>=0 && p.x<sourceSize.width && p.y<sourceSize.height,"Detection outside source frame"); return p;
}
Observation assignPieces(const Camera& camera,const Board& board,const std::string& frameId,
                         const std::vector<Detection>& detections,const ImageTransform& transform,
                         DetectionSpace space,bool clear,double minScore,double maxDistance) {
    Observation o; o.cameraId=camera.id; o.frameId=frameId; o.pieces.fill(Unknown); o.confidence.fill(0);
    require(minScore>=0 && minScore<=1 && maxDistance>0 && maxDistance<0.5,"Invalid assignment thresholds");
    if(!board.geometryValid || !board.orientationValid || board.cameraId!=camera.id || !camera.sameGeometry(board.geometryCamera) || frameId.empty() || frameId!=board.frameId || transform.sourceSize!=camera.imageSize) {
        o.issues.push_back("geometry/orientation/camera/frame mismatch"); return o; }
    if(clear) o.pieces.fill(Empty); else o.issues.push_back("scene not declared clear");
    std::array<bool,Cells> occupied{}; auto inverse=board.gridToPixel.inv();
    for(const auto& d:detections) {
        if(d.classId<0 || d.classId>=14 || !std::isfinite(d.confidence) || d.confidence<minScore || d.confidence>1 || d.bbox.width<=0 || d.bbox.height<=0) {
            o.issues.push_back("invalid or uncertain detection"); continue; }
        try {
            auto raw=transform.toOriginal({d.bbox.x+d.bbox.width/2,d.bbox.y+d.bbox.height/2},space);
            auto grid=project(inverse,camera.geometricPixel(raw)); int c=(int)std::lround(grid.x),r=(int)std::lround(grid.y);
            if(c<0 || c>=Columns || r<0 || r>=Rows || cv::norm(grid-cv::Point2d(c,r))>maxDistance) { o.issues.push_back("piece outside grid gate"); continue; }
            int i=r*Columns+c;
            if(occupied[i]) { o.pieces[i]=Unknown; o.confidence[i]=0; o.issues.push_back("multiple pieces assigned to one intersection"); }
            else { o.pieces[i]=d.classId; o.confidence[i]=d.confidence; occupied[i]=true; }
        } catch(const std::exception& e) { o.issues.push_back(e.what()); }
    }
    o.valid=o.issues.empty(); return o;
}
Stabilizer::Stabilizer(int frames,double tolerance):needed_(frames),tolerance_(tolerance) {
    require(frames>=2 && tolerance>=0,"Invalid stabilizer parameters"); previous_.pieces.fill(Unknown);
}
void Stabilizer::reset() { count_=0; previous_=Observation{}; previousBoard_=Board{}; }
Observation Stabilizer::update(const Board& board,const Observation& input) {
    auto result=input;
    if(!input.valid || !board.geometryValid || !board.orientationValid || input.cameraId!=board.cameraId || input.frameId!=board.frameId) { reset(); result.valid=false; return result; }
    if(input.frameId==previous_.frameId) { result.valid=false; result.issues.push_back("repeated frame is not new evidence"); return result; }
    bool same=count_>0 && input.cameraId==previous_.cameraId && input.pieces==previous_.pieces;
    for(int i:{0,8,81,89}) if(cv::norm(board.intersections[i]-previousBoard_.intersections[i])>tolerance_) same=false;
    count_=same?count_+1:1; previous_=input; previousBoard_=board;
    result.valid=count_>=needed_; if(!result.valid) result.issues.push_back("waiting for stable distinct frames"); return result;
}
std::optional<TargetXY> gridTarget(const PlaneMap& map,const Board& board,int c,int r,const std::string& currentFrameId) {
    if(!map.accepted || !board.geometryValid || !board.orientationValid || map.camera.id!=board.cameraId || !map.camera.sameGeometry(board.geometryCamera) || currentFrameId.empty() || currentFrameId!=board.frameId || c<0 || c>=Columns || r<0 || r>=Rows) return std::nullopt;
    auto pixel=board.intersections[r*Columns+c]; if(!contains(map.validPixelHull,pixel)) return std::nullopt;
    return TargetXY{c,r,project(map.pixelToBase,pixel),map.planeZmm,map.id,currentFrameId};
}
cv::Mat drawBoard(const cv::Mat& image,const Board& board,const Observation* o) {
    auto out=image.clone(); if(!board.geometryValid) return out;
    for(int r=0;r<Rows;++r) for(int c=0;c<Columns;++c) {
        auto p=board.intersections[r*Columns+c]; cv::circle(out,p,3,{0,255,255},-1);
        std::string label=std::to_string(c)+","+std::to_string(r);
        if(o) { int v=o->pieces[r*Columns+c]; label+=v==Unknown?":?":v==Empty?":.":std::string(":")+PieceSymbols[v]; }
        cv::putText(out,label,p+cv::Point2d(4,-4),cv::FONT_HERSHEY_SIMPLEX,0.32,{0,0,255},1,cv::LINE_AA);
    }
    cv::putText(out,board.orientationValid?"ORIENTATION CONFIRMED":"ORIENTATION UNKNOWN - NO ROBOT TARGETS",{10,24},cv::FONT_HERSHEY_SIMPLEX,0.55,{0,0,255},2);
    return out;
}
}
