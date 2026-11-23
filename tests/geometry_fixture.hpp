#include "bench/legacy_site.hpp"
#pragma once
#include "vision/geometry.hpp"

namespace fixture
{
struct Input { cv::Mat points; };
inline Input input(const cv::Matx23d &model = cv::Matx23d(.1,0,0,0,.1,0))
{
    Input result;
    result.points.create(3,4,CV_64F);
    double pixels[3][2]={{100,600},{900,600},{100,60}};
    for(int i=0;i<3;++i)
    {
        cv::Vec2d base=model*cv::Vec3d(pixels[i][0],pixels[i][1],1);
        result.points.at<double>(i,0)=pixels[i][0];
        result.points.at<double>(i,1)=pixels[i][1];
        result.points.at<double>(i,2)=base[0];
        result.points.at<double>(i,3)=base[1];
    }
    return result;
}
inline void storage(const Input &input, cv::FileStorage &file)
{
    cv::FileStorage writer("memory",cv::FileStorage::WRITE|cv::FileStorage::MEMORY|cv::FileStorage::FORMAT_JSON);
    writer<<"points"<<input.points;
    file.open(writer.releaseAndGetString(),cv::FileStorage::READ|cv::FileStorage::MEMORY);
}
inline void initialize(const Input &input,scara::Geometry &geometry)
{
    cv::FileStorage file;
    storage(input,file);
    scara::Bench_geometry(file.root(),geometry);
}
inline scara::Geometry geometry(const Input &input=fixture::input())
{
    scara::Geometry result;
    initialize(input,result);
    return result;
}
}
