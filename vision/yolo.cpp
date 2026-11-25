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
// 从标签的左下边缘向上画底色，返回文字布局用的左上角。
static cv::Point drawLabelBackground(cv::Mat &image, cv::Point bottomLeft, int height, const cv::Scalar &color)
{
    const int width = 44;
    int x = std::max(0, std::min(bottomLeft.x, image.cols - width));
    int y = std::max(0, bottomLeft.y - height);
    cv::rectangle(image, {x, y, width, height}, color, cv::FILLED);
    return {x, y};
}

cv::Mat Vision_draw(const cv::Mat &bgr, const std::vector<Detection> &detections)
{
    static cv::FontFace font("uni"); // OpenCV 5 内置的中文字体。
    // 名字按模型的类别编号排列，供预览直接取出相应棋名。
    static const char *const names[YoloClasses] = {
        "黑将", "黑士", "黑象", "黑马", "黑车", "黑炮", "黑卒",
        "红帅", "红仕", "红相", "红马", "红车", "红炮", "红兵"
    };
    // 在副本上绘制，保留原图供棋盘归格和对照。
    cv::Mat result = bgr.clone();
    const cv::Scalar white = cv::Scalar::all(255);
    cv::Rect bounds(0, 0, result.cols, result.rows);
    for (const Detection &piece : detections)
    {
        // 画棋子框。跳过画面外的框和没有对应棋名的类别。
        cv::Rect box = cv::Rect(piece.bbox) & bounds;
        if (box.empty() || piece.classId < 0 || piece.classId >= YoloClasses) continue;
        cv::Scalar color = piece.classId < 7 ? cv::Scalar(255, 180, 65) : cv::Scalar(65, 85, 245);
        cv::rectangle(result, box, color, 2);

        // 框上方写棋名，标签与框留 1 像素间隔。
        cv::Point namePos = drawLabelBackground(result, {box.x, box.y - 1}, 24, color);
        cv::putText(result, names[piece.classId], namePos + cv::Point(2, 19), white, font, 18);

        // 框底部写得分，保留两位小数。
        cv::Point scorePos = drawLabelBackground(result, {box.x, box.br().y}, 16, color);
        cv::putText(result, cv::format("%.2f", piece.confidence), scorePos + cv::Point(3, 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, white, 1, cv::LINE_AA);
    }
    return result;
}
} // namespace scara
