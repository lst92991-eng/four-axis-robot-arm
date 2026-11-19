// Field adapter for the existing game loop on this bench. Python owns the USB and ADB handles.
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "app/game_files.hpp"
#include "vision/capture.hpp"
#include "vision/io.hpp"
#include "vision/yolo.hpp"

#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace scara { void Bench_start(const GameConfig &config, Session &session); }
#define Match_start Bench_start
#define main scara_usb_match
#include "app/game_main.cpp"
#undef main
#undef Match_start

namespace scara
{
namespace
{
struct ReleasePy
{
    void operator()(PyObject *value) const { Py_XDECREF(value); }
};
using Ref = std::unique_ptr<PyObject, ReleasePy>;

PyObject *field = nullptr;
std::string streamUrl;
cv::Mat labels;
bool cameraOpen = false;
unsigned frameNumber = 0;

Ref require(PyObject *result)
{
    if (result) return Ref(result);
    if (PyErr_ExceptionMatches(PyExc_KeyboardInterrupt))
    {
        PyErr_Clear();
        gameRunning = 0;
        throw std::runtime_error("对局已停止");
    }
    PyObject *type = nullptr, *error = nullptr, *trace = nullptr;
    PyErr_Fetch(&type, &error, &trace);
    Ref ownedType(type), ownedError(error), ownedTrace(trace);
    Ref message(error ? PyObject_Str(error) : nullptr);
    const char *description = message ? PyUnicode_AsUTF8(message.get()) : nullptr;
    std::string text = description ? description : "USB-CAN 实机设备调用失败";
    PyErr_Clear();
    throw std::runtime_error(text);
}

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

void Runtime_open(const GameConfig &config, const std::string &modelPath)
{
    Files_open(config.workDir, false);
    streamUrl = config.cameraUrl;
    labels = cv::imread(config.labelPath, cv::IMREAD_GRAYSCALE);
    if (labels.cols != AtlasWidth || labels.rows != YoloClasses * LabelHeight)
        throw std::runtime_error("标签图不完整");
    Vision_setup();
    require(PyObject_CallMethod(field, "open", "s", modelPath.c_str()));
}

bool Runtime_close()
{
    try
    {
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

bool Motion_stop()
{
    auto stopped = require(PyObject_CallMethod(field, "stop", nullptr));
    return PyObject_IsTrue(stopped.get()) == 1;
}

void Bench_start(const GameConfig &config, Session &session)
{
    auto saved = require(PyObject_CallMethod(field, "resume_history", nullptr));
    const char *history = PyUnicode_AsUTF8(saved.get());
    if (!history) require(nullptr);
    if (!*history)
    {
        Match_start(config, session);
        return;
    }

    chess::Game restored;
    chess::Board start;
    chess::initialBoard(start);
    if (!chess::Game_reset(restored, start, true, config.humanRed))
        throw std::runtime_error("续局起始盘面无效");
    std::istringstream moves(history);
    std::string text;
    while (moves >> text)
    {
        chess::Move move;
        chess::Board after;
        if (!chess::parseMove(text, move) || !chess::applyMove(restored.board, restored.redToMove, move, after, restored.error))
            throw std::runtime_error("无法重放已有走法 " + text);
        chess::Game_commit(restored, after, move);
    }
    session.game = restored;
    auto pending = require(PyObject_CallMethod(field, "resume_pending", nullptr));
    const char *pendingMove = PyUnicode_AsUTF8(pending.get());
    if (!pendingMove) require(nullptr);
    chess::Board expected = restored.board;
    chess::Move move;
    if (*pendingMove)
    {
        std::vector<std::string> legal;
        std::string reply;
        Engine_query(restored, legal, reply);
        if (std::count(legal.begin(), legal.end(), pendingMove) != 1 || !chess::parseMove(pendingMove, move) ||
            restored.board[move.toRow][move.toCol] != '.' ||
            !chess::applyMove(restored.board, restored.redToMove, move, expected, restored.error))
            throw std::runtime_error("原计划应手无法继续");
        cv::Point2d from, to;
        std::vector<PickStep> steps;
        Geometry_target(session.layout, move.fromCol, Rows - 1 - move.fromRow, from);
        Geometry_target(session.layout, move.toCol, Rows - 1 - move.toRow, to);
        if (!Pick_plan(session.arm, session.heights, from, to, false, steps))
            throw std::runtime_error("原计划取放点不可达");
        // The stopped pickup has fresh encoder evidence. Resume with its down position, then return to park.
        steps.erase(steps.begin(), steps.begin() + 2);
        PickStep park;
        park.action = PickAction::Move;
        park.target = {session.heights.travelDeg, config.park.j2Deg, config.park.j3MotorDeg};
        steps.push_back(park);
        session.pending = true;
        Match_publishStatus("executing", session);
        bool completed = false;
        Robot_execute("resume_pickup", pendingMove, steps, completed);
        if (!completed) throw std::runtime_error("未完成原取放动作");
        auto rows = require(PyList_New(Rows));
        for (int row = 0; row < Rows; ++row)
        {
            auto text = require(PyUnicode_FromStringAndSize(expected[row].data(), Columns));
            PyList_SET_ITEM(rows.get(), row, text.release());
        }
        require(PyObject_CallMethod(field, "expect_board", "O", rows.get()));
    }
    Match_start(config, session);
    if (expected != session.observed.board) throw std::runtime_error("实拍盘面与已走历史不符");
    if (*pendingMove) chess::Game_commit(restored, expected, move);
    session.game = std::move(restored);
    session.pending = false;
}

void Vision_observe(const Geometry &layout, std::string &frameId, std::vector<Detection> &pieces)
{
    if (!gameRunning) throw std::runtime_error("对局已停止");
    if (!cameraOpen)
    {
        Camera_open(streamUrl);
        cameraOpen = true;
    }
    try
    {
        cv::Mat source, input;
        Camera_read(source);
        Vision_prepare(source, input);
        frameId = std::to_string(++frameNumber);
        auto packet = require(PyObject_CallMethod(field, "infer", "y#s", input.ptr<char>(),
                                                   Py_ssize_t(input.total() * input.elemSize()), frameId.c_str()));
        char *data = nullptr;
        Py_ssize_t size = 0;
        if (PyBytes_AsStringAndSize(packet.get(), &data, &size) < 0)
            require(nullptr);
        uint32_t headerSize = 0;
        if (size < 4) throw std::runtime_error("NPU回复不完整");
        std::memcpy(&headerSize, data, sizeof(headerSize));
        if (headerSize == 0 || headerSize > 16384 || size_t(size) < 4 + size_t(headerSize))
            throw std::runtime_error("NPU元数据长度错误");
        cv::FileStorage meta(std::string(data + 4, headerSize), cv::FileStorage::READ | cv::FileStorage::MEMORY);
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
            if (shape.size() != 4 || int(shape[0]) != 1 || int(shape[1]) != width ||
                int(shape[2]) != width || int(shape[3]) != YoloChannels || int(attr["type"]) != 2 ||
                int(attr["fmt"]) != 1 || bytes > size_t(size) - offset ||
                (stride != 0 && stride != width) || !(scale > 0))
                throw std::runtime_error("NPU张量格式不符");
            heads[i] = {reinterpret_cast<const int8_t *>(data + offset), size_t(width) * YoloChannels,
                        int(attr["zero_point"]), scale};
            offset += bytes;
        }
        if (offset != size_t(size)) throw std::runtime_error("NPU回复有多余数据");
        Vision_decode(heads, pieces);
        cv::Mat predicted, grid, preview;
        Vision_draw(source, pieces, labels, predicted);
        Geometry_draw(source, layout, grid);
        cv::hconcat(predicted, grid, preview);
        std::vector<unsigned char> jpeg;
        if (!cv::imencode(".jpg", preview, jpeg)) throw std::runtime_error("无法编码预览");
        if (!Runtime_publish("preview.jpg", std::string(jpeg.begin(), jpeg.end())))
            throw std::runtime_error("无法保存当前画面");

        // The bench has a real piece 0.417 cell off center. For assignment only,
        // pass the unique closest cell within 0.45; keep the decoded boxes in the preview.
        for (auto &piece : pieces)
        {
            const cv::Point2d center(piece.bbox.x + piece.bbox.width / 2, piece.bbox.y + piece.bbox.height / 2);
            cv::Vec2d gridPoint = layout.pixelToGrid * cv::Vec3d(center.x, center.y, 1);
            int col = int(std::round(gridPoint[0])), row = int(std::round(gridPoint[1]));
            if (col < 0 || col >= Columns || row < 0 || row >= Rows) continue;
            double error = std::hypot(gridPoint[0] - col, gridPoint[1] - row);
            if (error <= .35 || error > .45) continue;
            const auto intersection = layout.pixels[row * Columns + col];
            piece.bbox.x += intersection.x - center.x;
            piece.bbox.y += intersection.y - center.y;
        }
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

void Engine_query(const chess::Game &game, std::vector<std::string> &legal, std::string &choice)
{
    auto query = require(PyObject_CallMethod(field, "engine", "ssi", game.baseFen.c_str(), game.history.c_str(),
                                             int(game.redToMove != game.humanRed)));
    PyObject *moves = PyTuple_GetItem(query.get(), 0);
    PyObject *best = PyTuple_GetItem(query.get(), 1);
    if (!moves || !best) require(nullptr);
    legal.clear();
    for (Py_ssize_t n = 0; n < PyList_Size(moves); ++n)
    {
        const char *uci = PyUnicode_AsUTF8(PyList_GetItem(moves, n));
        if (!uci) require(nullptr);
        legal.emplace_back(uci);
    }
    const char *result = PyUnicode_AsUTF8(best);
    if (!result) require(nullptr);
    choice = result;
}

void Operator_read(const std::string &phase, const std::string &prompt, Command &command, std::string *move)
{
    auto answer = require(PyObject_CallMethod(field, "command", "ss", phase.c_str(), prompt.c_str()));
    const char *text = PyUnicode_AsUTF8(answer.get());
    if (!text) require(nullptr);
    if (!Command_parse(text, command, move)) throw std::runtime_error("未识别的终端命令");
    if (command == Command::Quit)
    {
        gameRunning = 0;
        throw std::runtime_error("对局已停止");
    }
}

void Robot_execute(const std::string &purpose, const std::string &move, const std::vector<PickStep> &steps,
                   bool &success)
{
    auto actions = require(PyList_New(steps.size()));
    for (size_t i = 0; i < steps.size(); ++i)
    {
        const PickStep &s = steps[i];
        auto item = require(Py_BuildValue("(iidddi)", int(s.action), int(s.axes), s.target[0], s.target[1],
                                          s.target[2], s.durationMs));
        PyList_SET_ITEM(actions.get(), i, item.release());
    }
    auto done = require(PyObject_CallMethod(field, "execute", "ssO", purpose.c_str(), move.c_str(), actions.get()));
    success = PyObject_IsTrue(done.get()) == 1;
}
} // namespace scara

extern "C" __declspec(dllexport) int run_usb_game(PyObject *callbacks, char *configPath, char *modelPath)
{
    scara::field = callbacks;
    scara::gameRunning = 1;
    char name[] = "scara_usb_game";
    char *argv[] = {name, configPath, modelPath};
    int result = scara_usb_match(3, argv);
    scara::field = nullptr;
    return result;
}
