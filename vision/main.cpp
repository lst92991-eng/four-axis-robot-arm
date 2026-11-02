// 第一步：把相机的一帧NV12数据转换成BGR彩色图，再保存为照片。
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cstdio>

int main()
{
    // 与本板采集脚本一致，单位是像素。
    const int width = 1280;
    const int height = 960;
    const int frameBytes = width * height * 3 / 2;

    // NV12的Y数据占一份，UV数据占半份，所以用1.5倍行数存放字节。
    cv::Mat nv12(height * 3 / 2, width, CV_8UC1);

    // 读取camera.nv12中的第一帧，直接放入Mat的数据区。
    std::FILE* file = std::fopen("camera.nv12", "rb");
    if (file == nullptr) {
        return 1;
    }
    const size_t bytesRead = std::fread(nv12.data, 1, frameBytes, file);
    std::fclose(file);
    if (bytesRead != frameBytes) {
        return 1;
    }

    // 这一句完成格式转换：NV12输入，BGR输出。
    cv::Mat bgr;
    cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);

    // capture.sh会准备live目录；保存照片后就结束这一步。
    if (!cv::imwrite("live/source.jpg", bgr)) {
        return 1;
    }
    return 0;
}
