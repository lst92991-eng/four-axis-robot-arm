#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <iostream>
int main(int argc,char **argv) {
    if(argc>1) {
        cv::aruco::ArucoDetector detector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
        auto image=cv::imread(argv[1]); std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detector.detectMarkers(image,corners,ids);
        std::cout<<"OpenCV "<<CV_VERSION<<" decoded:";
        for(int id:ids)std::cout<<' '<<id;
        std::cout<<'\n';return ids.size()==4?0:1;
    }
    std::cout<<"{\"opencv\":\""<<CV_VERSION<<"\",\"markers\":[";
    for(int id=0;id<4;++id) {
        cv::Mat marker;
        cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50),id,6,marker);
        if(id)std::cout<<',';
        std::cout<<'[';
        for(int r=0;r<6;++r) {
            if(r)std::cout<<','; std::cout<<'[';
            for(int c=0;c<6;++c) {if(c)std::cout<<',';std::cout<<int(marker.at<unsigned char>(r,c)!=0);}
            std::cout<<']';
        }
        std::cout<<']';
    }
    std::cout<<"]}\n";
}
