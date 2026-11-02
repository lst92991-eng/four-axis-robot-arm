#include <exception>
#include <iostream>
#include <opencv2/core.hpp>
void testDevices();
void testMotion();
void testVisionChess();
void testApp();
int main() {
    try {
        std::cout << "OpenCV " << cv::getVersionString() << '\n';
        testDevices(); testMotion(); testVisionChess(); testApp();
        std::cout << "All teaching tests passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
