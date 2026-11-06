#include "vision/geometry.hpp"
#include "vision/io.hpp"
#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <iostream>
#include <functional>
#include <cmath>

using namespace scara;
namespace {
int checks=0;
void check(bool v,const char* message) {++checks;if(!v)throw std::runtime_error(message);}
void rejects(const std::function<void()>& f) {bool thrown=false;try{f();}catch(const std::exception&){thrown=true;}check(thrown,"Invalid input was accepted");}
Camera camera() {Camera c;c.id="synthetic-test-only";c.imageSize={640,480};return c;}
const cv::Matx33d BaseH(0.4,0.02,100,-0.01,0.5,-80,0.0001,-0.00005,1);
PlaneMap plane() {
    std::vector<PointPair> fit,validation;
    for(auto p:std::vector<cv::Point2d>{{20,20},{620,20},{620,460},{20,460},{320,70},{180,330}})fit.push_back({p,project(BaseH,p)});
    for(auto p:std::vector<cv::Point2d>{{150,180},{320,260},{450,300}})validation.push_back({p,project(BaseH,p)});
    return calibratePlane(camera(),"synthetic-map-do-not-use-on-robot",12,0.001,fit,validation);
}
Board board(const std::string& frame="1") {
    return boardFromCorners(camera(),frame,{{{150,80},{500,100},{480,400},{130,380}}},true);
}
void calibrationTests() {
    auto p=plane();check(p.accepted && p.checkError.maxMm<0.001,"Exact planar fit failed");
    auto path=(std::filesystem::temp_directory_path()/std::filesystem::u8path("scara_几何测试.yml")).u8string();
    savePlane(path,p);auto loaded=loadPlane(path);check(loaded.id==p.id && loaded.accepted,"Persistence failed");
    auto bad=p.checkPoints;bad[0].baseXY.x+=10;check(!calibratePlane(p.camera,p.id,p.planeZmm,p.allowedErrorMm,p.fitPoints,bad).accepted,"Validation error not rejected");
    rejects([&]{auto v=p.checkPoints;v[0]=p.fitPoints[0];calibratePlane(p.camera,p.id,0,1,p.fitPoints,v);});
    rejects([&]{std::vector<PointPair> line;for(int i=0;i<4;++i)line.push_back({{50.+i*50,100},{double(i),0}});calibratePlane(p.camera,p.id,0,1,line,p.checkPoints);});
    auto altered=readBytes(path);auto at=altered.find("pixel_to_base:");check(at!=std::string::npos,"Missing saved matrix");
    // Matrix corruption is checked against persisted independent sample pairs on load.
    auto corrupt=p;corrupt.pixelToBase(0,2)+=30;savePlane(path,corrupt);rejects([&]{loadPlane(path);});savePlane(path,p);
    auto c=camera();c.K=(cv::Mat_<double>(3,3)<<500,0,320,0,500,240,0,0,1);c.distortion=(cv::Mat_<double>(1,5)<<-.1,.02,0,0,0);
    std::vector<cv::Point3d> objects{{-.3,.2,1}};std::vector<cv::Point2d> raw;
    cv::projectPoints(objects,cv::Vec3d(),cv::Vec3d(),c.K,c.distortion,raw);
    check(cv::norm(c.geometricPixel(raw[0])-cv::Point2d(170,340))<.001,"Undistorted pixels are not in K pixel coordinates");
}
void targetTests() {
    auto p=plane();auto b=board();auto t=gridTarget(p,b,3,4,"1");check(t.has_value(),"Target missing");
    check(cv::norm(t->baseMm-project(BaseH,b.intersections[39]))<.001,"Wrong target XY");
    check(!gridTarget(p,b,9,0,"1") && !gridTarget(p,b,0,0,"old"),"Bounds/stale frame accepted");
    auto unknown=b;unknown.orientationValid=false;check(!gridTarget(p,unknown,3,4,"1"),"Unoriented board generated target");
    auto wrong=b;wrong.cameraId="different-camera";check(!gridTarget(p,wrong,3,4,"1"),"Different camera accepted");
    wrong=b;wrong.geometryCamera.imageSize.width+=1;check(!gridTarget(p,wrong,3,4,"1"),"Changed numeric camera profile accepted under same ID");
    auto limited=p;limited.validPixelHull={{20,20},{80,20},{80,80},{20,80}};check(!gridTarget(limited,b,3,4,"1"),"Calibration extrapolation accepted");
    double a=.15,co=std::cos(a),si=std::sin(a);cv::Matx33d move(co,-si,55,si,co,-25,0,0,1);
    std::array<cv::Point2d,4> corners;int j=0;for(int i:{0,8,89,81})corners[j++]=project(move,b.intersections[i]);
    auto moved=boardFromCorners(camera(),"2",corners,true);auto mt=gridTarget(p,moved,3,4,"2");
    check(mt && cv::norm(mt->baseMm-project(BaseH,project(move,b.intersections[39])))<.001,"Moved board incorrectly requires/reuses old robot calibration");
    check(cv::norm(mt->baseMm-t->baseMm)>1,"Moved board target did not change");
    rejects([&]{boardFromCorners(camera(),"x",{{{100,100},{400,400},{400,100},{100,400}}},true);});
}
void assignmentTests() {
    ImageTransform transform;transform.sourceSize={640,480};transform.roi={100,40,300,200};
    const cv::Point2d point(30,40);const std::array<cv::Point2d,4> rotated{{point,{159,30},{269,159},{40,269}}};
    int i=0;
    for(int rotation:{0,90,180,270}) {transform.clockwiseDegrees=rotation;auto p=rotated[i++];
        check(cv::norm(transform.toOriginal(p,DetectionSpace::RotatedRoi)-cv::Point2d(130,80))<1e-9,"ROI inverse rotation failed");
        cv::Size size=(rotation==90 || rotation==270)?cv::Size(200,300):cv::Size(300,200);transform.resizedSize={size.width*2,size.height*2};transform.padding={11,17};
        check(cv::norm(transform.toOriginal(p*2+transform.padding,DetectionSpace::Network)-cv::Point2d(130,80))<1e-9,"Letterbox inverse failed"); }
    auto b=board();auto center=b.intersections[39];Detection d{12,.9,{center.x-12,center.y-12,24,24}};
    auto o=assignPieces(camera(),b,"1",{d},transform,DetectionSpace::Original,true);
    check(o.valid && o.pieces[39]==12 && o.pieces[0]==Empty,"Grid assignment failed");
    auto hidden=assignPieces(camera(),b,"1",{d},transform,DetectionSpace::Original,false);check(!hidden.valid && hidden.pieces[0]==Unknown,"Occluded cell treated as empty");
    auto collision=assignPieces(camera(),b,"1",{d,d},transform,DetectionSpace::Original,true);check(!collision.valid && collision.pieces[39]==Unknown,"Collision silently resolved");
    auto stale=assignPieces(camera(),b,"wrong",{d},transform,DetectionSpace::Original,true);check(!stale.valid,"Mixed frames accepted");
    auto p=project(b.gridToPixel,{3.48,4});d.bbox={p.x-12,p.y-12,24,24};check(!assignPieces(camera(),b,"1",{d},transform,DetectionSpace::Original,true).valid,"Off-grid piece accepted");
    Stabilizer stable(3);check(!stable.update(b,o).valid,"Single frame stable");check(!stable.update(b,o).valid,"Repeated frame counted");
    b.frameId=o.frameId="2";check(!stable.update(b,o).valid,"Two frames stable");b.frameId=o.frameId="3";check(stable.update(b,o).valid,"Three frames did not stabilize");
    b.frameId=o.frameId="4";b.intersections[0].x+=10;check(!stable.update(b,o).valid,"Board motion failed to reset stability");
}
void boardTests() {
    Camera c;c.id="synthetic-grid-only";c.imageSize={800,700};cv::Mat image(c.imageSize,CV_8UC3,cv::Scalar(235,230,215));
    cv::rectangle(image,{128,68,424,474},{20,20,20},4);
    for(int x=0;x<9;++x) {int u=140+x*50;if(x==0 || x==8)cv::line(image,{u,80},{u,530},{30,30,30},2);
        else {cv::line(image,{u,80},{u,280},{30,30,30},2);cv::line(image,{u,330},{u,530},{30,30,30},2);} }
    for(int r=0;r<10;++r)cv::line(image,{140,80+r*50},{540,80+r*50},{30,30,30},2);
    for(int y:{80,430}){cv::line(image,{290,y},{390,y+100},{30,30,30},2);cv::line(image,{390,y},{290,y+100},{30,30,30},2);}
    cv::putText(image,"RIVER-A",{200,314},cv::FONT_HERSHEY_SIMPLEX,.55,{15,15,15},2);
    auto b=locateBoard(image,c,"auto");check(b.geometryValid,"Synthetic natural-grid localization failed");check(!b.orientationValid,"Symmetric grid silently chose a red side");
    double maxError=0;for(auto p:b.intersections){double nearest=1e9;for(int r=0;r<10;++r)for(int col=0;col<9;++col)nearest=std::min(nearest,cv::norm(p-cv::Point2d(140+col*50,80+r*50)));maxError=std::max(maxError,nearest);}
    std::cout << "synthetic grid maximum nearest-point error: " << maxError << " px\n";check(maxError<4,"Wrong lattice selected");
    auto exact=boardFromCorners(c,"ref",{{{140,530},{540,530},{540,80},{140,80}}},true);
    auto ref=makeOrientationReference(image,c,exact,{65,218,220,40});auto flipped=confirmOrientation(exact,true);flipped.orientationValid=false;
    auto oriented=orientByReference(image,c,flipped,ref);check(oriented.orientationValid && cv::norm(oriented.intersections[0]-cv::Point2d(140,530))<.01,"Printed-feature orientation failed");
    auto autoOriented=orientByReference(image,c,b,ref);check(autoOriented.orientationValid && cv::norm(autoOriented.intersections[0]-cv::Point2d(140,530))<4,"Auto-grid chirality/orientation mismatch");
    auto symmetric=ref;symmetric.patch=cv::Mat::ones(ref.patch.size(),CV_8U)*120;
    check(!orientByReference(image,c,exact,symmetric).orientationValid,"Ambiguous texture accepted");
    cv::Mat blank(c.imageSize,CV_8UC3,cv::Scalar::all(255));check(!locateBoard(blank,c,"blank").geometryValid,"Blank image accepted as board");
    auto affine=cv::getRotationMatrix2D({340,305},20,1);affine.at<double>(0,2)+=40;affine.at<double>(1,2)+=20;cv::Mat moved;
    cv::warpAffine(image,moved,affine,c.imageSize,cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar::all(240));
    check(locateBoard(moved,c,"rotated").geometryValid,"Rotated natural grid not found");
}
}
int main() {
    try{calibrationTests();targetTests();assignmentTests();boardTests();std::cout<<"PASS "<<checks<<" geometry checks; no hardware linked.\n";return 0;}
    catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<" checks: "<<e.what()<<'\n';return 1;}
}
