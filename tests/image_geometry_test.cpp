#include "vision/geometry.hpp"
#include "vision/io.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <iostream>
#include <algorithm>

// Offline repeatability against known image transforms. Not a physical millimetre accuracy test.
int main(int argc,char** argv) {
    try {
        if(argc!=3 && argc!=4)throw std::runtime_error("Usage: scara_geometry_image_test original.png output-directory [orientation.yml]");
        auto bytes=scara::readBytes(argv[1]);std::vector<unsigned char> encoded(bytes.begin(),bytes.end());
        auto image=cv::imdecode(encoded,cv::IMREAD_COLOR);if(image.empty())throw std::runtime_error("No source image");
        std::optional<scara::OrientationReference> orientation;
        if(argc==4)orientation=scara::loadOrientationReference(argv[3]);
        scara::Camera camera;camera.id=orientation?orientation->cameraId:"offline-image-repeatability-only";camera.imageSize=image.size();
        auto reference=scara::locateBoard(image,camera,"reference");if(!reference.geometryValid)throw std::runtime_error("Reference grid not found");
        if(orientation){reference=scara::orientByReference(image,camera,reference,*orientation);if(!reference.orientationValid)throw std::runtime_error("Reference orientation ambiguous");}
        cv::Point2d center;for(auto p:reference.intersections)center+=p;center*=1.0/scara::Cells;
        std::filesystem::path directory=std::filesystem::u8path(argv[2]);std::filesystem::create_directories(directory);
        auto report=scara::memoryStorage(true);report<<"physical_accuracy_validated"<<0<<"cases"<<"[";
        for(int angle:{0,10,-10,90,180,270}) {
            auto affine=cv::getRotationMatrix2D(center,angle,0.85);affine.at<double>(0,2)+=15;affine.at<double>(1,2)-=25;
            cv::Mat moved;cv::warpAffine(image,moved,affine,image.size(),cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar::all(230));
            auto actual=scara::locateBoard(moved,camera,"rotation-"+std::to_string(angle));
            if(!actual.geometryValid)throw std::runtime_error("Grid not found at rotation "+std::to_string(angle));
            if(orientation){actual=scara::orientByReference(moved,camera,actual,*orientation);if(!actual.orientationValid)throw std::runtime_error("Printed-feature orientation ambiguous at "+std::to_string(angle));}
            double maximum=0,squared=0;
            for(int i=0;i<scara::Cells;++i) {auto p=reference.intersections[i];
                cv::Point2d expected(affine.at<double>(0,0)*p.x+affine.at<double>(0,1)*p.y+affine.at<double>(0,2),affine.at<double>(1,0)*p.x+affine.at<double>(1,1)*p.y+affine.at<double>(1,2));
                double nearest=1e9;
                if(orientation)nearest=cv::norm(expected-actual.intersections[i]);
                else for(auto q:actual.intersections)nearest=std::min(nearest,cv::norm(expected-q));
                maximum=std::max(maximum,nearest);squared+=nearest*nearest;
            }
            std::string name="rotation-"+std::to_string(angle)+".png";std::vector<unsigned char> png;
            cv::imencode(".png",scara::drawBoard(moved,actual),png);scara::writeBytes((directory/name).u8string(),{png.begin(),png.end()});
            std::cout<<angle<<" deg: grid score="<<actual.gridScore<<" max="<<maximum<<" rms="<<std::sqrt(squared/scara::Cells)<<" px\n";
            report<<"{"<<"rotation_degrees"<<angle<<"max_repeatability_px"<<maximum<<"rms_repeatability_px"<<std::sqrt(squared/scara::Cells)<<"orientation_confirmed"<<(int)actual.orientationValid<<"}";
            if(maximum>7.0)throw std::runtime_error("Wrong/unstable grid lattice at rotation "+std::to_string(angle));
        }
        report<<"]";scara::writeBytes((directory/"repeatability.json").u8string(),report.releaseAndGetString());
        std::cout<<"PASS saved-image grid repeatability; no robot data or motion.\n";return 0;
    }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
