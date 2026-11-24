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
    check(failed, "Invalid calibration did not stop initialization");
    check(geometry.cell[0][0].xy == cv::Point2d(), "Failed setup reused an old target");
}
void calibrationTests()
{
    const cv::Matx23d model(.4,.02,100,-.01,.5,-80);
    Geometry geometry;
    cv::Matx23d legacyMatrix;
    cv::FileStorage legacy;
    fixture::storage(fixture::input(model),legacy);
    Bench_geometry(legacy.root(),geometry,&legacyMatrix);
    check(cv::norm(legacyMatrix-model,cv::NORM_INF)<1e-10,"Six coefficients differ");
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
            cv::Point2d actual = geometry.cell[row][column].xy;
            check(cv::norm(actual-cv::Point2d(10+10*column,60-6*(9-row)))<1e-9,"Wrong physical target");
        }
    auto moved=fixture::input();
    for(int i=0;i<3;++i)
    {
        moved.points.at<double>(i,0)+=20;
        moved.points.at<double>(i,1)-=20;
        moved.points.at<double>(i,2)+=2;
        moved.points.at<double>(i,3)-=2;
    }
    auto shifted=fixture::geometry(moved);
    check(cv::norm(shifted.cell[5][3].xy-cv::Point2d(42,34))<1e-9,
          "Moved board failed to update targets");
    cv::Mat source(720,1280,CV_8UC3,cv::Scalar(0)), image;
    Geometry_draw(source,geometry,image);
    check(image.size()==source.size() && cv::norm(image,source)>0,"Grid overlay missing");
    check(cv::norm(source)==0,"Drawing changed the source image");
}
void observationTests()
{
    auto geometry=fixture::geometry();
    cv::Point2d center = geometry.cell[4][3].pixel + cv::Point2d(5,-3);
    cv::Point2d fixedTarget = geometry.cell[4][3].xy;
    Detection piece{12,.9,{center.x-12,center.y-12,24,24}};
    chess::Board result;
    bool valid = Geometry_assign(geometry, {piece}, result);
    check(valid && result[4][3]=='C' && result[0][0]=='.', "Single frame assignment failed");
    check(geometry.cell[4][3].xy == fixedTarget, "Off-center piece changed the fixed target");
    valid = Geometry_assign(geometry, {piece,piece}, result);
    check(!valid && result[4][3]=='?', "Duplicate detection accepted");
    valid = Geometry_assign(geometry, {piece}, result, false);
    check(!valid && result[4][3]=='?', "Occlusion exposed a usable observation");
    piece.bbox.x += 48;
    check(!Geometry_assign(geometry, {piece}, result), "Off-grid center accepted");
    valid = Geometry_assign(geometry, {}, result);
    check(valid && result[4][3]=='.', "Empty board retained stale pieces");
}
void placementOffsetTests()
{
    cv::FileStorage writer("memory",cv::FileStorage::WRITE|cv::FileStorage::MEMORY|cv::FileStorage::FORMAT_JSON);
    writer << "points" << fixture::input().points;
    writer << "placement_offsets_mm" << "[" << "[" << 4 << 2 << -5.0 << -2.5 << "]" << "]";
    cv::FileStorage reader(writer.releaseAndGetString(),cv::FileStorage::READ|cv::FileStorage::MEMORY);
    Geometry geometry;
    Bench_geometry(reader.root(),geometry);
    check(cv::norm((geometry.origin.xy + 7 * geometry.rowStep.xy + 4 * geometry.columnStep.xy)-cv::Point2d(50,48))<1e-9,
          "Placement trim changed nominal board geometry");
    check(cv::norm(geometry.cell[7][4].xy-cv::Point2d(45,45.5))<1e-9,
          "Pick/place target omitted the configured trim");
    check(cv::norm(geometry.cell[7][5].xy-cv::Point2d(60,48))<1e-9,
          "Placement trim affected a neighboring square");
}
}
int main()
{
    try
    {
        calibrationTests(); targetTests(); observationTests(); placementOffsetTests();
        std::cout<<"PASS "<<count<<" geometry checks; no hardware linked.\n";
        return 0;
    }
    catch(const std::exception &e)
    {
        std::cerr<<"FAIL after "<<count<<" checks: "<<e.what()<<'\n';
        return 1;
    }
}
