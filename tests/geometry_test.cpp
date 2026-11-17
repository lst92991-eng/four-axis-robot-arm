#include "geometry_fixture.hpp"
#include <functional>
#include <iostream>
#include <limits>

using namespace scara;
namespace
{
int count = 0;
void check(bool value, const char *message)
{
    ++count;
    if (!value) throw std::runtime_error(message);
}
void rejects(const fixture::Input &input)
{
    Geometry geometry = fixture::geometry();
    bool failed = false;
    try { fixture::initialize(input, geometry); }
    catch (const std::exception &) { failed = true; }
    check(failed && !geometry.ready, "Invalid initialization kept a usable layout");
    cv::Point2d target(99,99);
    check(!Geometry_target(geometry,0,0,target) && target == cv::Point2d(), "Failed setup reused an old target");
}
void calibrationTests()
{
    const cv::Matx23d model(.4,.02,100,-.01,.5,-80);
    auto geometry=fixture::geometry(fixture::input(model));
    check(geometry.ready,"Three-point initialization failed");
    check(cv::norm(geometry.pixelToBase-model,cv::NORM_INF)<1e-10,"Six coefficients differ");
    auto bad=fixture::input(); bad.points=bad.points.rowRange(0,2).clone(); rejects(bad);
    bad=fixture::input(); bad.points=bad.points.colRange(0,3).clone(); rejects(bad);
    bad=fixture::input(); bad.points.convertTo(bad.points,CV_32F); rejects(bad);
    bad=fixture::input(); bad.points.at<double>(0,0)=std::numeric_limits<double>::quiet_NaN(); rejects(bad);
    bad=fixture::input(); bad.points.col(1).setTo(100); rejects(bad);
}
void targetTests()
{
    auto geometry=fixture::geometry();
    for(int row=0;row<Rows;++row)
        for(int column=0;column<Columns;++column)
        {
            cv::Point2d actual;
            check(Geometry_target(geometry,column,row,actual), "Missing one of 90 targets");
            check(cv::norm(actual-cv::Point2d(10+10*column,60-6*row))<1e-9,"Wrong physical target");
        }
    cv::Point2d target(99,99);
    check(!Geometry_target(geometry,9,0,target) && target==cv::Point2d(),"Invalid column reused old XY");
    check(!Geometry_target(geometry,0,-1,target),"Invalid row accepted");
    auto moved=fixture::input();
    for(int i=0;i<3;++i)
    {
        moved.points.at<double>(i,0)+=20;
        moved.points.at<double>(i,1)-=20;
        moved.points.at<double>(i,2)+=2;
        moved.points.at<double>(i,3)-=2;
    }
    auto shifted=fixture::geometry(moved);
    check(Geometry_target(shifted,3,4,target) && cv::norm(target-cv::Point2d(42,34))<1e-9,
          "Moved board failed to update targets");
    cv::Mat source(720,1280,CV_8UC3,cv::Scalar(0)), image;
    Geometry_draw(source,geometry,image);
    check(image.size()==source.size() && cv::norm(image,source)>0,"Grid overlay missing");
    check(cv::norm(source)==0,"Drawing changed the source image");
}
void observationTests()
{
    auto geometry=fixture::geometry();
    cv::Point2d center=geometry.pixels[39];
    Detection piece{12,.9,{center.x-12,center.y-12,24,24}};
    Observation result;
    Geometry_observe(geometry,"1",{piece},true,result);
    check(!result.valid && result.pieces[39]==12 && geometry.stableFrames==1,"First observation failed");
    Geometry_observe(geometry,"1",{piece},true,result);
    check(!result.valid && geometry.stableFrames==1,"Repeated frame advanced confirmation");
    Geometry_observe(geometry,"2",{piece},true,result);
    check(!result.valid && geometry.stableFrames==2,"Two frames already stable");
    Geometry_observe(geometry,"3",{piece},true,result);
    check(result.valid && result.pieces[0]==Empty,"Three frames failed to stabilize");
    Geometry_observe(geometry,"4",{piece,piece},true,result);
    check(!result.valid && result.pieces[39]==Unknown && geometry.stableFrames==0,"Duplicate detection accepted");
    Geometry_observe(geometry,"5",{piece},false,result);
    check(!result.valid && result.pieces[39]==Unknown,"Occlusion exposed a usable observation");
    Geometry_observe(geometry,"",{piece},true,result);
    check(!result.valid && geometry.stableFrames==0,"Missing frame ID accepted");
    piece.bbox.x+=48;
    Geometry_observe(geometry,"6",{piece},true,result);
    check(!result.valid,"Off-grid center accepted");
    for(int i=7;i<10;++i) Geometry_observe(geometry,std::to_string(i),{},true,result);
    check(result.valid && result.pieces[39]==Empty,"Empty board retained stale pieces");
    fixture::initialize(fixture::input(),geometry);
    Geometry_observe(geometry,"10",{},true,result);
    check(!result.valid && geometry.stableFrames==1,"New layout retained old stability");
}
}
int main()
{
    try
    {
        calibrationTests(); targetTests(); observationTests();
        std::cout<<"PASS "<<count<<" geometry checks; no hardware linked.\n";
        return 0;
    }
    catch(const std::exception &e)
    {
        std::cerr<<"FAIL after "<<count<<" checks: "<<e.what()<<'\n';
        return 1;
    }
}
