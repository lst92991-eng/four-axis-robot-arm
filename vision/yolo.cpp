//处理识别图像，解码模型输出，并生成棋子检测列表和预览图
#include "yolo.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <stdexcept>

namespace scara
{
//---------图像与模型参数区---------
//相机原图尺寸，用于核对每帧输入
static cv::Size FrameSize;
//Roi是原图裁剪区域，InputArea是缩放图在640×640输入中的位置
static cv::Rect Roi, InputArea;
//裁剪图到模型输入的缩放比例，预处理和坐标还原共用
static double Scale;
//候选框最低得分，以及去重时使用的交并比阈值
constexpr float ScoreThreshold = .50f, NmsThreshold = .45f;
// 类别标签的背景宽高和得分背景的高度，单位像素。
constexpr int LabelWidth = 44, LabelHeight = 24, ScoreHeight = 16;
//三路输出各自对应的锚框宽高，单位为模型输入图的像素
static const cv::Size2f Anchors[YoloHeads][YoloAnchors] = {
    {{10, 13}, {16, 30}, {33, 23}},
    {{30, 61}, {62, 45}, {59, 119}},
    {{116, 90}, {156, 198}, {373, 326}},
};

//候选表的列顺序，前四列为位置编码，第五列为目标分数
enum PredictionColumn
{
    CenterX,
    CenterY,
    WidthCode,
    HeightCode,
    ObjectScore,
    ClassBegin
};

static void readCandidates(const YoloHead &head, int index, std::vector<Detection> &candidates);
static void selectCandidates(std::vector<Detection> &candidates, std::vector<Detection> &selected);
static bool restoreBox(cv::Rect2d &box);

//---------初始化区---------
void Vision_setup(cv::Size frameSize, cv::Rect roi)
{
    //保存固定机位使用的原图尺寸和识别区域
    FrameSize = frameSize;
    Roi = roi;
    //取宽高两个缩放比例中的较小值，使整块裁剪图放进640×640输入
    Scale = std::min(double(YoloSize) / roi.width, double(YoloSize) / roi.height);
    //将缩放尺寸取整为实际像素数
    InputArea.width = cvRound(roi.width * Scale);
    InputArea.height = cvRound(roi.height * Scale);
    //缩放图居中放置，剩余边缘作为填充区域
    InputArea.x = (YoloSize - InputArea.width) / 2;
    InputArea.y = (YoloSize - InputArea.height) / 2;
}

//---------图像预处理区---------
void Vision_prepare(const cv::Mat &bgr, cv::Mat &input)
{
    //核对8位三通道原图和固定分辨率
    if (bgr.type() != CV_8UC3 || bgr.size() != FrameSize) throw std::runtime_error("Unexpected camera frame");
    //裁剪识别区域，并将OpenCV的BGR通道顺序转换为模型使用的RGB
    cv::Mat rgb;
    cv::cvtColor(bgr(Roi), rgb, cv::COLOR_BGR2RGB);
    //创建640×640输入图，填充像素值114
    input = cv::Mat(YoloSize, YoloSize, CV_8UC3, cv::Scalar::all(114));
    //取得有效画面区域的内存视图，缩放结果直接写入输入图
    cv::Mat picture = input(InputArea);
    //用双线性插值缩放裁剪图，外侧区域保留填充值
    cv::resize(rgb, picture, InputArea.size(), 0, 0, cv::INTER_LINEAR);
}

//---------输出解码区---------
void Vision_decode(const YoloHead heads[YoloHeads], std::vector<Detection> &boxes)
{
    boxes.clear();
    // 第一步，从三路输出读出模型坐标下的候选框。
    std::vector<Detection> candidates;
    for (int i = 0; i < YoloHeads; ++i)
        readCandidates(heads[i], i, candidates);

    // 第二步，按得分保留高分框，同分沿用读取顺序。
    std::vector<Detection> selected;
    selectCandidates(candidates, selected);

    // 第三步，还原到原图像素坐标，裁掉落在补边里的框。
    for (Detection piece : selected)
        if (restoreBox(piece.bbox)) boxes.push_back(piece);
}

//---------识别绘制区---------
void Vision_draw(const cv::Mat &bgr, const std::vector<Detection> &detections, cv::Mat &result)
{
    static cv::FontFace font("uni"); // OpenCV 5 内置的中文字体。
    // 名字按模型的类别编号排列，供预览直接取出相应棋名。
    static const char *const names[YoloClasses] = {
        "黑将", "黑士", "黑象", "黑马", "黑车", "黑炮", "黑卒",
        "红帅", "红仕", "红相", "红马", "红车", "红炮", "红兵"
    };
    //复制原尺寸画面，后续绘制写入预览图
    result = bgr.clone();
    //绘图边界为整张原图
    cv::Rect bounds(0, 0, result.cols, result.rows);
    for (const Detection &piece : detections)
    {
        //将检测框转换为整数像素，并裁切到可绘制区域
        cv::Rect box = cv::Rect(piece.bbox) & bounds;
        if (box.empty() || piece.classId < 0 || piece.classId >= YoloClasses) continue;
        //按红黑双方选择显示颜色，画出检测框
        cv::Scalar color = piece.classId < 7 ? cv::Scalar(255, 180, 65) : cv::Scalar(65, 85, 245);
        cv::rectangle(result, box, color, 2);
        //中文标签放在检测框上方，位置限制在图像范围内
        int x = std::clamp(box.x, 0, std::max(0, result.cols - LabelWidth));
        int y = std::clamp(box.y - LabelHeight - 1, 0, std::max(0, result.rows - LabelHeight));
        cv::rectangle(result, {x, y, LabelWidth, LabelHeight}, color, cv::FILLED);
        cv::putText(result, names[piece.classId], {x + 2, y + 19}, cv::Scalar::all(255), font, 18);
        //在检测框底部显示得分，保留两位小数
        int scoreY = std::clamp(box.y + box.height - ScoreHeight, 0, std::max(0, result.rows - ScoreHeight));
        cv::rectangle(result, {x, scoreY, LabelWidth, ScoreHeight}, color, cv::FILLED);
        std::string score = cv::format("%.2f", piece.confidence);
        cv::putText(result, score, {x + 3, scoreY + 12}, cv::FONT_HERSHEY_SIMPLEX, .4,
                    cv::Scalar::all(255), 1, cv::LINE_AA);
    }
}

//---------候选表解码区---------
//将一路输出整理成候选表，解算达到阈值的行
static void readCandidates(const YoloHead &head, int index, std::vector<Detection> &candidates)
{
    //本路输出的网格边长，分别为80、40或20
    int size = YoloGrid[index];
    //每个网格单元对应的输入图跨度，分别为8、16或32像素
    int stride = YoloSize / size;
    //按实际行跨度读取INT8内存，OpenCV转换后生成连续的浮点工作区
    cv::Mat raw(size, size * YoloChannels, CV_8S, const_cast<int8_t *>(head.data), head.rowBytes);
    cv::Mat values;
    raw.convertTo(values, CV_32F);
    //先减零点再乘系数，还原每个输出值
    values -= head.zeroPoint;
    values *= head.scale;
    //每行整理为一个候选框，前5列是位置与目标分数，随后14列是类别分数
    values = values.reshape(1, size * size * YoloAnchors);
    for (int row = 0; row < values.rows; ++row)
    {
        const float *prediction = values.ptr<float>(row);
        // 每行先在 14 类中选最大类别分数，再乘同一行的目标分数，得到最终置信度。
        const float *classScores = prediction + ClassBegin;
        const float *best = std::max_element(classScores, classScores + YoloClasses);
        float score = prediction[ObjectScore] * *best;
        if (score < ScoreThreshold) continue;

        //每个网格点有3行候选，行号决定它的网格位置和锚框
        int cell = row / YoloAnchors;
        int x = cell % size, y = cell / size;
        int anchor = row % YoloAnchors;
        //中心位置由格内偏移和网格步长解算，单位为模型输入像素
        float offsetX = prediction[CenterX] * 2 - .5f;
        float offsetY = prediction[CenterY] * 2 - .5f;
        double centerX = (double(offsetX) + x) * stride;
        double centerY = (double(offsetY) + y) * stride;
        //宽高由预测倍数和本路锚框尺寸解算
        float widthFactor = prediction[WidthCode] * 2;
        float heightFactor = prediction[HeightCode] * 2;
        cv::Size2f anchorSize = Anchors[index][anchor];
        float width = widthFactor * widthFactor * anchorSize.width;
        float height = heightFactor * heightFactor * anchorSize.height;

        //填写本候选框的类别、得分和位置
        Detection candidate;
        candidate.classId = int(best - classScores);
        candidate.confidence = score;
        candidate.bbox = cv::Rect2d(centerX - width / 2, centerY - height / 2, width, height);
        candidates.push_back(candidate);
    }
}

//---------候选框去重区---------
static void selectCandidates(std::vector<Detection> &candidates, std::vector<Detection> &selected)
{
    //按得分降序处理，同分保持原读取顺序
    std::stable_sort(candidates.begin(), candidates.end(), [](const Detection &a, const Detection &b)
    {
        return a.confidence > b.confidence;
    });
    for (const Detection &candidate : candidates)
    {
        //逐个与已经保留的高分框比较
        bool duplicate = false;
        for (const Detection &kept : selected)
        {
            double overlap = (kept.bbox & candidate.bbox).area();
            double total = kept.bbox.area() + candidate.bbox.area() - overlap;
            // 交叠面积除以并集面积就是 IoU。超过阈值时，当前候选让位给已保留的高分框。
            if (overlap / std::max(total, 1e-9) > NmsThreshold)
            {
                duplicate = true;
                break;
            }
        }
        //通过检查后追加，原候选表始终按顺序读取
        if (!duplicate) selected.push_back(candidate);
    }
}

//---------坐标还原区---------
static bool restoreBox(cv::Rect2d &box)
{
    //去除补边并撤销缩放，得到原裁剪区域中的四条边
    double left = (box.x - InputArea.x) / Scale;
    double top = (box.y - InputArea.y) / Scale;
    double right = (box.x + box.width - InputArea.x) / Scale;
    double bottom = (box.y + box.height - InputArea.y) / Scale;
    //将边界限制在识别区域内，再加回原图偏移
    left = std::clamp(left, 0.0, double(Roi.width));
    top = std::clamp(top, 0.0, double(Roi.height));
    right = std::clamp(right, 0.0, double(Roi.width));
    bottom = std::clamp(bottom, 0.0, double(Roi.height));
    box = {left + Roi.x, top + Roi.y, right - left, bottom - top};
    //裁切后有面积的框进入最终结果
    return right > left && bottom > top;
}
} // namespace scara
