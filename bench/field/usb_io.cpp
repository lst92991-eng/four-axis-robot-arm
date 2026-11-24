// Windows USB-CAN 台架接入。C++ 负责视觉解码与对局，Python 持有 USB-CAN 和 ADB 连接。
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "app/site.hpp"
#include "scara_cad.hpp"
#include "vision/capture.hpp"
#include "vision/yolo.hpp"

#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace scara
{
namespace
{
//---------Python 对象与设备会话区---------
// Ref 负责释放返回给 C++ 的 Python 引用，作用域结束时将引用计数减一。
struct ReleasePy
{
    void operator()(PyObject *value) const { Py_XDECREF(value); }
};
using Ref = std::unique_ptr<PyObject, ReleasePy>;

// field 是本局传入的 Python 设备对象，接收板端推理、引擎、面板和 USB-CAN 动作的调用。
PyObject *field = nullptr;
std::string streamUrl;
bool cameraOpen = false;
unsigned frameNumber = 0;

// 把 Python 的返回对象交给 Ref 管理。调用失败则把原因带回 C++ 对局。
Ref require(PyObject *result)
{
    if (result) return Ref(result);
    // 键盘中断先清除运行标志，使上层沿退出流程停车。
    if (PyErr_ExceptionMatches(PyExc_KeyboardInterrupt))
    {
        PyErr_Clear();
        gameRunning = 0;
        throw std::runtime_error("对局已停止");
    }
    PyObject *type = nullptr, *error = nullptr, *trace = nullptr;
    // 取出异常对象并转成 UTF-8 消息，让恢复提示显示设备端的原因。
    PyErr_Fetch(&type, &error, &trace);
    Ref ownedType(type), ownedError(error), ownedTrace(trace);
    Ref message(error ? PyObject_Str(error) : nullptr);
    const char *description = message ? PyUnicode_AsUTF8(message.get()) : nullptr;
    std::string text = description ? description : "USB-CAN 实机设备调用失败";
    PyErr_Clear();
    throw std::runtime_error(text);
}

// 关闭本机取流连接后，再让 Python 回收板端推理会话。
void closeVision()
{
    if (cameraOpen)
    {
        Camera_close();
        cameraOpen = false;
    }
    require(PyObject_CallMethod(field, "close_vision", nullptr));
}
} // namespace

static void readHistory(const GameConfig &config, chess::Game &game);

//---------台架启动与收尾区---------
void Runtime_open(const GameConfig &config, const std::string &modelPath, Session &session)
{
    // 固定相机地址及本机 ROI 随安装参数传入，Python 准备板端设备。
    streamUrl = config.cameraUrl;
    Vision_setup({ImageWidth, ImageHeight}, config.visionRoi);
    require(PyObject_CallMethod(field, "open", "s", modelPath.c_str()));
    // 现场台架允许棋子中心偏离交点 0.45 格，随后检查本次是否要接续棋谱。
    session.layout.assignmentTolerance = .45;
    readHistory(config, session.game);
}

bool Runtime_close()
{
    try
    {
        // 先回收视觉会话，再由 Python 结束台架设备操作并返回停车结果。
        closeVision();
        auto stopped = require(PyObject_CallMethod(field, "close", nullptr));
        return PyObject_IsTrue(stopped.get()) == 1;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return false;
    }
}

// 将整组停车请求交给持有总线连接的 Python 执行端。
bool Motion_stop()
{
    auto stopped = require(PyObject_CallMethod(field, "stop", nullptr));
    return PyObject_IsTrue(stopped.get()) == 1;
}

//---------接续已确认棋局区---------
static void readHistory(const GameConfig &config, chess::Game &game)
{
    auto saved = require(PyObject_CallMethod(field, "resume_history", nullptr));
    const char *history = PyUnicode_AsUTF8(saved.get());
    if (!history) require(nullptr);
    if (!*history) return;

    // 用临时棋局逐手重放，从保存的起始盘面开始。
    chess::Game restored;
    chess::Board start;
    auto initial = require(PyObject_CallMethod(field, "resume_board", nullptr));
    if (!PyList_Check(initial.get()) || PyList_Size(initial.get()) != Rows)
        throw std::runtime_error("续局起始盘面行数有误");
    // 按黑方底线到红方底线的顺序读取十行，每行是九个棋子编码。
    for (int row = 0; row < Rows; ++row)
    {
        Py_ssize_t size = 0;
        const char *text = PyUnicode_AsUTF8AndSize(PyList_GetItem(initial.get(), row), &size);
        if (!text) require(nullptr);
        if (size != Columns) throw std::runtime_error("续局起始盘面列数有误");
        std::memcpy(start[row].data(), text, Columns);
    }
    if (!chess::Game_reset(restored, start, config.redToMove, config.humanRed))
        throw std::runtime_error("续局起始盘面无效");
    std::istringstream moves(history);
    std::string text;
    while (moves >> text)
    {
        if (!chess::Game_replay(restored, text)) throw std::runtime_error("无法重放已有走法 " + text);
    }
    // 完整棋谱重放成功后，才用它替换本局的已确认状态。
    game = std::move(restored);
}

//---------取帧、推理与预览区---------
void Vision_observe(const Geometry &layout, std::string &frameId, std::vector<Detection> &pieces)
{
    if (!gameRunning) return;
    if (!cameraOpen)
    {
        Camera_open(streamUrl);
        cameraOpen = true;
    }
    try
    {
        // source 保存 BGR 原图，input 保存送往 NPU 的 640×640 RGB 像素。
        cv::Mat source, input;
        try
        {
            // 取帧最多等待 5500 毫秒。发生取流异常时重新连接，再取一帧。
            Camera_read(source, 5500);
        }
        catch (const std::runtime_error &)
        {
            if (!gameRunning) return;
            std::cout << "相机取帧超时，正在重新连接。" << std::endl;
            Camera_open(streamUrl);
            Camera_read(source, 5500);
        }
        if (!gameRunning) return;
        // 给本帧分配标识，把 RGB 缓冲和帧号一起送给板端推理。
        Vision_prepare(source, input);
        frameId = std::to_string(++frameNumber);
        auto packet = require(PyObject_CallMethod(field, "infer", "y#s", input.ptr<char>(),
                                                   Py_ssize_t(input.total() * input.elemSize()), frameId.c_str()));
        char *data = nullptr;
        Py_ssize_t size = 0;
        if (PyBytes_AsStringAndSize(packet.get(), &data, &size) < 0)
            require(nullptr);
        // 回复的前 4 字节是元数据长度，随后是元数据文本和三段 INT8 张量。
        uint32_t headerSize = 0;
        if (size < 4) throw std::runtime_error("NPU回复不完整");
        std::memcpy(&headerSize, data, sizeof(headerSize));
        if (headerSize == 0 || headerSize > 16384 || size_t(size) < 4 + size_t(headerSize))
            throw std::runtime_error("NPU元数据长度错误");
        cv::FileStorage meta(std::string(data + 4, headerSize), cv::FileStorage::READ | cv::FileStorage::MEMORY);
        // 先核对回复帧号和输出个数，确保三个张量都来自本次请求。
        if (std::string(meta["frame_id"]) != frameId || meta["outputs"].size() != YoloHeads)
            throw std::runtime_error("NPU回复帧号或输出数不符");
        size_t offset = 4 + headerSize;
        YoloHead heads[YoloHeads];
        for (int i = 0; i < YoloHeads; ++i)
        {
            const int width = YoloGrid[i];
            auto attr = meta["outputs"][i];
            auto shape = attr["dims"];
            const int stride = int(attr["w_stride"]);
            const float scale = float(attr["scale"]);
            const size_t bytes = size_t(width) * width * YoloChannels;
            // 核对 NHWC 形状、INT8 类型、行宽和有效字节数，再把数据交给解码器。
            if (shape.size() != 4 || int(shape[0]) != 1 || int(shape[1]) != width ||
                int(shape[2]) != width || int(shape[3]) != YoloChannels || int(attr["type"]) != 2 ||
                int(attr["fmt"]) != 1 || bytes > size_t(size) - offset ||
                (stride != 0 && stride != width) || !(scale > 0))
                throw std::runtime_error("NPU张量格式不符");
            // 输出描述直接引用 packet 中的数据，本次解码在 packet 释放前完成。
            heads[i] = {reinterpret_cast<const int8_t *>(data + offset), size_t(width) * YoloChannels,
                        int(attr["zero_point"]), scale};
            offset += bytes;
        }
        if (offset != size_t(size)) throw std::runtime_error("NPU回复有多余数据");
        // 三个尺度的框在原图上解码。左图画类别与分数，右图画固定格点。
        Vision_decode(heads, pieces);
        cv::Mat predicted, grid, preview;
        Vision_draw(source, pieces, predicted);
        Geometry_draw(source, layout, grid);
        cv::hconcat(predicted, grid, preview);
        // 将双画面压成 JPEG，通过内存回调交给面板。
        std::vector<unsigned char> jpeg;
        if (!cv::imencode(".jpg", preview, jpeg)) throw std::runtime_error("无法编码预览");
        require(PyObject_CallMethod(field, "preview", "y#", jpeg.data(), Py_ssize_t(jpeg.size())));

        // 棋盘下沿的弃子落在盘外。保留原检测框供预览，只从盘面归格中移除。
        pieces.erase(std::remove_if(pieces.begin(), pieces.end(), [&](const Detection &piece) {
            const cv::Point2d center(piece.bbox.x + piece.bbox.width / 2, piece.bbox.y + piece.bbox.height / 2);
            double row, column;
            Geometry_rowCol(center - layout.origin.pixel, layout.rowStep.pixel, layout.columnStep.pixel, row, column);
            return column >= Columns - .5 && row >= -.5 && row < Rows - .5;
        }), pieces.end());

    }
    catch (...)
    {
        if (cameraOpen)
        {
            Camera_close();
            cameraOpen = false;
        }
        throw;
    }
}

//---------引擎查询区---------
void Engine_query(const chess::Game &game, chess::MoveList &legal, chess::Move &choice)
{
    // 把起始 FEN、已确认棋谱和是否轮到电脑交给 Python 查询皮卡鱼。
    auto query = require(PyObject_CallMethod(field, "engine", "ssi", game.baseFen.c_str(), game.history.c_str(),
                                             int(game.redToMove != game.humanRed)));
    PyObject *moves = PyTuple_GetItem(query.get(), 0);
    PyObject *best = PyTuple_GetItem(query.get(), 1);
    if (!moves || !best) require(nullptr);
    // 把引擎的四字符走法逐条转回 C++ 数字行列，再单独读取本轮应手。
    legal.clear();
    for (Py_ssize_t n = 0; n < PyList_Size(moves); ++n)
    {
        const char *uci = PyUnicode_AsUTF8(PyList_GetItem(moves, n));
        if (!uci) require(nullptr);
        chess::Move move;
        if (!chess::parseMove(uci, move)) throw std::runtime_error("引擎合法走法格式错误");
        legal.push_back(move);
    }
    const char *result = PyUnicode_AsUTF8(best);
    if (!result) require(nullptr);
    choice = {};
    if (*result && !chess::parseMove(result, choice)) throw std::runtime_error("引擎应手格式错误");
}

//---------面板输出与输入区---------
// Python 直接取得两张盘面和数字走法。图像走独立的字节回调。
void Operator_show(const char *phase, const Session &s, const std::string &message)
{
    std::string board, observed;
    // 正式盘面和观测盘面都按行展开，各用 90 个字符交给显示端。
    for (int row = 0; row < Rows; ++row)
    {
        board.append(s.game.board[row].data(), Columns);
        observed.append(s.observed.board[row].data(), Columns);
    }
    auto legal = require(PyList_New(0));
    // 等待玩家选棋时，把每手合法走法写成两组（行,列），供面板高亮目标。
    if (std::strcmp(phase, "wait_human") == 0)
        for (const chess::Move &m : s.legal)
        {
            auto item = require(Py_BuildValue("((ii)(ii))", m.fromRow, m.fromCol, m.toRow, m.toCol));
            if (PyList_Append(legal.get(), item.get()) < 0) require(nullptr);
        }
    require(PyObject_CallMethod(field, "state", "ssssssiiiO", phase, message.c_str(), s.lastFrameId.c_str(),
                                s.game.history.c_str(), board.c_str(), observed.c_str(), int(s.game.result),
                                int(s.pending), int(s.observed.valid), legal.get()));
}

void Operator_read(const std::string &phase, const std::string &prompt, Command &command, chess::Move *move)
{
    PyObject *value = PyObject_CallMethod(field, "command", "ss", phase.c_str(), prompt.c_str());
    if (!value && (PyErr_ExceptionMatches(PyExc_EOFError) || PyErr_ExceptionMatches(PyExc_KeyboardInterrupt)))
    {
        PyErr_Clear();
        command = Command::Quit;
        gameRunning = 0;
        return;
    }
    auto answer = require(value);
    const char *text = PyUnicode_AsUTF8(answer.get());
    if (!text) require(nullptr);
    if (!Command_parse(text, command, move)) throw std::runtime_error("未识别的终端命令");
    if (command == Command::Quit) gameRunning = 0;
}

//---------USB-CAN 动作执行区---------
bool Robot_execute(const std::string &purpose, const chess::Move &move, const std::vector<PickStep> &steps)
{
    if (!gameRunning) return false;
    // 按计划顺序传出动作类型、轴掩码、三个驱动角（度）和停留时间（毫秒）。
    auto actions = require(PyList_New(steps.size()));
    for (size_t i = 0; i < steps.size(); ++i)
    {
        const PickStep &s = steps[i];
        auto item = require(Py_BuildValue("(iidddi)", int(s.action), int(s.axes), s.target[0], s.target[1],
                                          s.target[2], s.durationMs));
        PyList_SET_ITEM(actions.get(), i, item.release());
    }
    // 走法文字用于标记这次搬棋，Python 执行的机械目标来自 actions。
    std::string moveText;
    chess::uci(move, moveText); // 准备阶段的记录留空。
    auto done = require(PyObject_CallMethod(field, "execute", "ssO", purpose.c_str(), moveText.c_str(), actions.get()));
    return PyObject_IsTrue(done.get()) == 1;
}
} // namespace scara

//---------DLL 入口区---------
static void loadFieldSite(const char *text, size_t size, scara::GameConfig &config, scara::Session &session)
{
    config.thinkMs = 650;
    config.engineDir = "/userdata/scara-live-match-20261117/engine";
    scara::Site_parse(std::string_view(text, size), config, session);
    if (config.cameraUrl.empty()) throw std::runtime_error("缺少 vision.camera_url");
}

// 两个入口接收同一份 TOML 快照，防止开局前编辑文件导致驱动端参数不一致。
extern "C" __declspec(dllexport) PyObject *usb_installation(const char *text, size_t size)
{
    using namespace scara;
    GameConfig cfg;
    Session s;
    try { loadFieldSite(text, size, cfg, s); }
    catch (const std::exception &error) { PyErr_SetString(PyExc_ValueError, error.what()); return nullptr; }
    // 交付机械参考与臂长、Z 工作层、观察位、三轴限幅、吸合时间和 CAD 采样数据。
    return Py_BuildValue("{s:(dddddd),s:(dd),s:(ddd),s:(dd),s:(ddd),s:(ddd),s:i,s:y#,s:i,s:s}",
        "arm", s.arm.j2ReferenceDeg, s.arm.j3ReferenceMotorDeg, s.arm.shoulderReferenceRad, s.arm.elbowReferenceRad,
        s.arm.j2JointPerDrive, s.arm.j3JointPerDrive,
        "arm_lengths_mm", s.arm.upperMm, s.arm.forearmMm,
        "z", s.heights.pickupDeg, s.heights.travelDeg, s.heights.releaseDeg,
        "park", s.park.j2Deg, s.park.j3MotorDeg,
        "velocity", cfg.motion.velocity[0], cfg.motion.velocity[1], cfg.motion.velocity[2],
        "acceleration", cfg.motion.acceleration[0], cfg.motion.acceleration[1], cfg.motion.acceleration[2],
        "grip_ms", s.heights.gripDwellMs, "cad", cadSamples, Py_ssize_t(sizeof(cadSamples)),
        "think_ms", cfg.thinkMs, "engine_directory", cfg.engineDir.c_str());
}

// Python 传入本局设备对象与模型路径，DLL 使用与板端相同的 Match_run。
extern "C" __declspec(dllexport) int run_usb_game(PyObject *callbacks, const char *modelPath, const char *text, size_t size)
{
    scara::GameConfig config;
    scara::Session session;
    scara::field = callbacks;
    int result = 1;
    try
    {
        loadFieldSite(text, size, config, session);
        result = scara::Match_run(config, session, modelPath);
    }
    catch (const std::exception &error) { PyErr_SetString(PyExc_RuntimeError, error.what()); }
    // 对局结束后解除设备对象与本次调用的关联。
    scara::field = nullptr;
    return result;
}
