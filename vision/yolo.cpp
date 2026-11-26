// 裁出模型需要的 RGB 输入图，并在预览画面上标注棋子。
#include "yolo.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace scara
{
//---------图像预处理区---------
cv::Mat Vision_prepare(const cv::Mat &bgr, cv::Point origin)
{
    // 按原图坐标裁出棋盘，转换为模型训练时使用的 RGB 顺序。
    cv::Rect crop(origin, cv::Size(YoloSize, YoloSize));
    cv::Mat rgb;
    cv::cvtColor(bgr(crop), rgb, cv::COLOR_BGR2RGB);
    return rgb;
}

//---------识别绘制区---------
// 画标签的底色，返回文字布局用的左上角。
// topLeft --> +----------------------+
//             |                      |  height
//             +----------------------+
//             <------- 90 px ------->
static cv::Point drawLabelBox(cv::Mat &image, cv::Point topLeft, int height, const cv::Scalar &color)
{
    // 一行棋名和得分共用 90 像素宽的底色。
    const int width = 90;

    // 标签右边超出画面时，把它向左挪到刚好能放下的位置。
    if (topLeft.x + width > image.cols)
        topLeft.x = image.cols - width;
    // 左上角的横坐标最小为 0，对应画面的左边缘。
    topLeft.x = std::max(topLeft.x, 0);
    // 标签伸出画面顶部时，把它移到第 0 行。
    topLeft.y = std::max(topLeft.y, 0);

    // 根据调整后的左上角和标签宽高确定矩形范围。
    cv::Rect labelBox(topLeft, cv::Size(width, height));
    // 用棋子框的颜色填满这个矩形。
    cv::rectangle(image, labelBox, color, cv::FILLED);
    // 把实际左上角返回给调用者，后面从这里往内留白再写字。
    return topLeft;
}

void Vision_draw(cv::Mat &image, const std::vector<Detection> &detections)
{
    static cv::FontFace font("uni"); // OpenCV 5 内置的中文字体。
    // 名字按模型的类别编号排列，供预览直接取出相应棋名。
    static const char *const names[YoloClasses] = {
        "黑将", "黑士", "黑象", "黑马", "黑车", "黑炮", "黑卒",
        "红帅", "红仕", "红相", "红马", "红车", "红炮", "红兵"
    };
    const cv::Scalar white = cv::Scalar::all(255);
    for (const Detection &piece : detections)
    {
        // 解码已将棋子框还原到原图。绘制时使用整数像素。
        cv::Rect box(piece.bbox);
        // 黑方用蓝色框，红方用红色框。
        cv::Scalar color = piece.classId < 7 ? cv::Scalar(255, 180, 65) : cv::Scalar(65, 85, 245);
        cv::rectangle(image, box, color, 2);

        // 将棋名和两位小数的得分写在同一条标签上。
        const int labelHeight = 24;
        cv::Point labelTopLeft(box.x, box.y - labelHeight);
        cv::Point labelPos = drawLabelBox(image, labelTopLeft, labelHeight, color);
        std::string label = cv::format("%s %.2f", names[piece.classId], piece.confidence);
        cv::putText(image, label, labelPos + cv::Point(2, 19), white, font, 18);
    }
}
} // namespace scara
