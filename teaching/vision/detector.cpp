// Copyright (c) 2023 Rockchip Electronics Co., Ltd. All Rights Reserved.
// Adapted NHWC YOLOv5 decoding from RKNN Model Zoo v1.6.0, Apache-2.0.
// Source and full license: vision/SOURCES.md, vision/Rockchip-Apache-2.0.txt.
// Changes: 14 classes, native tensor stride checks, OpenCV preprocessing,
// bounded candidates, standard-library sorting and explicit resource cleanup.

#include "detector.hpp"
#include <rknn_api.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <new>

namespace {
constexpr int ClassCount = 14;
constexpr int ValuesPerAnchor = 5 + ClassCount;
constexpr int HeadCount = 3;
constexpr float ScoreThreshold = 0.50f;
constexpr float IouThreshold = 0.45f;
constexpr size_t CandidateLimit = 2048;
// 与导出脚本共同固定；训练禁用 autoanchor，导出前再次核对实际 anchors。
constexpr float Anchors[3][6] = {
    {10, 13, 16, 30, 33, 23},
    {30, 61, 62, 45, 59, 119},
    {116, 90, 156, 198, 373, 326}
};

struct OutputTensor {
    rknn_tensor_attr attr{};
    rknn_tensor_mem* memory = nullptr;
    int width = 0, height = 0, channels = 0;
    size_t rowBytes = 0;
};

float dequantize(int8_t value, const rknn_tensor_attr& attr) {
    return (static_cast<int>(value) - attr.zp) * attr.scale;
}

float intersectionOverUnion(const Detection& a, const Detection& b) {
    const float width = std::max(0.0f, std::min(a.x2, b.x2) - std::max(a.x1, b.x1));
    const float height = std::max(0.0f, std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
    const float intersection = width * height;
    const float area = (a.x2 - a.x1) * (a.y2 - a.y1)
                     + (b.x2 - b.x1) * (b.y2 - b.y1) - intersection;
    return area > 0 ? intersection / area : 0.0f;
}
}  // namespace

struct Detector {
    rknn_context context = 0;
    rknn_tensor_attr input{};
    rknn_tensor_mem* inputMemory = nullptr;
    OutputTensor outputs[HeadCount];
    int width = 0, height = 0;
    size_t inputRowBytes = 0;
};

void detector_close(Detector* detector) {
    if (!detector) return;
    // memory 由 RKNN 创建，也由 RKNN 销毁；不能再 free(memory)。
    for (auto& output : detector->outputs)
        if (output.memory) rknn_destroy_mem(detector->context, output.memory);
    if (detector->inputMemory)
        rknn_destroy_mem(detector->context, detector->inputMemory);
    if (detector->context) rknn_destroy(detector->context);
    delete detector;
}

Detector* detector_load(const char* modelPath, std::string& error) {
    error.clear();
    auto* detector = new (std::nothrow) Detector;
    if (!detector) { error = "Not enough memory for detector."; return nullptr; }
    auto fail = [&](const char* message) -> Detector* {
        error = message;
        detector_close(detector);
        return nullptr;
    };
    if (!modelPath || !modelPath[0]) return fail("Model path is empty.");
    // Mini Runtime 支持路径加载；不在用户空间再保留一份模型文件副本。
    if (rknn_init(&detector->context, const_cast<char*>(modelPath), 0, 0, nullptr) < 0)
        return fail("RKNN model load failed; check model, NPU driver and runtime 1.6.0.");

    rknn_input_output_num count{};
    if (rknn_query(detector->context, RKNN_QUERY_IN_OUT_NUM, &count, sizeof(count)) < 0
        || count.n_input != 1 || count.n_output != HeadCount)
        return fail("Expected one RGB input and three YOLOv5 output heads.");
    if (rknn_query(detector->context, RKNN_QUERY_NATIVE_INPUT_ATTR,
                   &detector->input, sizeof(detector->input)) < 0)
        return fail("Cannot query native RKNN input.");
    auto& input = detector->input;
    if (input.n_dims != 4 || input.fmt != RKNN_TENSOR_NHWC
        || input.dims[0] != 1 || input.dims[3] != 3
        || input.dims[1] != 640 || input.dims[2] != 640)
        return fail("Expected fixed 640x640 RGB model input.");
    detector->height = input.dims[1];
    detector->width = input.dims[2];
    const size_t inputStride = input.w_stride ? input.w_stride : input.dims[2];
    detector->inputRowBytes = inputStride * 3;
    if (inputStride < input.dims[2] || inputStride > 1024
        || input.size_with_stride > 4 * 1024 * 1024
        || detector->inputRowBytes * detector->height > input.size_with_stride)
        return fail("Invalid native input stride or allocation size.");
    // RGB888 由转换模型内的 mean=0/std=255 处理，禁止调用者重复归一化。
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;
    input.pass_through = 0;
    detector->inputMemory = rknn_create_mem(detector->context, input.size_with_stride);
    if (!detector->inputMemory || !detector->inputMemory->virt_addr
        || detector->inputMemory->size < input.size_with_stride
        || rknn_set_io_mem(detector->context, detector->inputMemory, &input) < 0)
        return fail("Cannot allocate or bind RKNN input memory.");

    for (int head = 0; head < HeadCount; ++head) {
        auto& output = detector->outputs[head];
        rknn_tensor_attr logical{};
        logical.index = head;
        if (rknn_query(detector->context, RKNN_QUERY_OUTPUT_ATTR, &logical, sizeof(logical)) < 0
            || logical.n_dims != 4 || logical.dims[0] != 1)
            return fail("Cannot query logical YOLOv5 output shape.");
        // 先核对逻辑通道57，再按 native 属性读取物理布局，不能把填充当类别。
        const bool nchw = logical.fmt == RKNN_TENSOR_NCHW;
        if (!nchw && logical.fmt != RKNN_TENSOR_NHWC)
            return fail("Unsupported logical output layout.");
        const int classesAndBoxes = logical.dims[nchw ? 1 : 3];
        output.height = logical.dims[nchw ? 2 : 1];
        output.width = logical.dims[nchw ? 3 : 2];
        const int stride = 8 << head;
        if (classesAndBoxes != ValuesPerAnchor * 3
            || output.height != detector->height / stride
            || output.width != detector->width / stride)
            return fail("Model must contain 14 classes and ordered stride-8/16/32 heads.");

        output.attr.index = head;
        if (rknn_query(detector->context, RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR,
                       &output.attr, sizeof(output.attr)) < 0)
            return fail("Cannot query native NHWC output.");
        const auto& native = output.attr;
        if (native.n_dims != 4 || native.dims[0] != 1 || native.fmt != RKNN_TENSOR_NHWC
            || native.type != RKNN_TENSOR_INT8 || native.qnt_type != RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC
            || !std::isfinite(native.scale) || native.scale <= 0
            || native.dims[1] < static_cast<uint32_t>(output.height)
            || native.dims[2] < static_cast<uint32_t>(output.width)
            || native.dims[1] > 640 || native.dims[2] > 640
            || native.dims[3] < ValuesPerAnchor * 3 || native.dims[3] > 128
            || native.size_with_stride > 8 * 1024 * 1024)
            return fail("Unsupported native quantized output layout.");
        const size_t rowWidth = native.w_stride ? native.w_stride : native.dims[2];
        if (rowWidth < native.dims[2] || rowWidth > 1024)
            return fail("Unsupported native output row stride.");
        // native 维度和显式 w_stride 描述数据；size_with_stride 是分配容量。
        // 容量更大不等于逐像素通道变多，不能据容量反推 C 的跨度。
        output.channels = native.dims[3];
        output.rowBytes = rowWidth * output.channels;
        const size_t lastByte = (output.height - 1) * output.rowBytes
                             + (output.width - 1) * output.channels + ValuesPerAnchor * 3;
        if (lastByte > native.size_with_stride)
            return fail("Native output stride exceeds the tensor buffer.");
        output.memory = rknn_create_mem(detector->context, native.size_with_stride);
        if (!output.memory || !output.memory->virt_addr || output.memory->size < lastByte
            || rknn_set_io_mem(detector->context, output.memory, &output.attr) < 0)
            return fail("Cannot allocate or bind RKNN output memory.");
    }
    return detector;
}

bool detector_detect(Detector* detector, const cv::Mat& rgb,
                     std::vector<Detection>& result, std::string& error) {
    result.clear();
    error.clear();
    if (!detector || rgb.empty() || rgb.type() != CV_8UC3) {
        error = "Detector requires an initialized model and RGB888 image.";
        return false;
    }
    try {
        const float ratio = std::min(static_cast<float>(detector->width) / rgb.cols,
                                     static_cast<float>(detector->height) / rgb.rows);
        const int resizedWidth = std::max(1, cvRound(rgb.cols * ratio));
        const int resizedHeight = std::max(1, cvRound(rgb.rows * ratio));
        const int left = (detector->width - resizedWidth) / 2;
        const int top = (detector->height - resizedHeight) / 2;
        cv::Mat resized;
        cv::resize(rgb, resized, cv::Size(resizedWidth, resizedHeight), 0, 0, cv::INTER_LINEAR);
        auto* destination = static_cast<uint8_t*>(detector->inputMemory->virt_addr);
        std::memset(destination, 114, detector->inputMemory->size);
        for (int row = 0; row < resizedHeight; ++row)
            std::memcpy(destination + (top + row) * detector->inputRowBytes + left * 3,
                        resized.ptr(row), resizedWidth * 3);
        // CPU 写入后交给 NPU；同步调用同时适用于缓存映射的张量内存。
        if (rknn_mem_sync(detector->context, detector->inputMemory, RKNN_MEMORY_SYNC_TO_DEVICE) < 0
            || rknn_run(detector->context, nullptr) < 0) {
            error = "NPU inference failed.";
            return false;
        }
        std::vector<Detection> candidates;
        for (int head = 0; head < HeadCount; ++head) {
            const auto& output = detector->outputs[head];
            if (rknn_mem_sync(detector->context, output.memory, RKNN_MEMORY_SYNC_FROM_DEVICE) < 0) {
                error = "Cannot synchronize NPU output memory.";
                return false;
            }
            const auto* data = static_cast<const int8_t*>(output.memory->virt_addr);
            const float stride = static_cast<float>(8 << head);
            for (int row = 0; row < output.height; ++row) {
                for (int col = 0; col < output.width; ++col) {
                    for (int anchor = 0; anchor < 3; ++anchor) {
                        const int8_t* values = data + row * output.rowBytes
                                            + col * output.channels + anchor * ValuesPerAnchor;
                        const float objectness = dequantize(values[4], output.attr);
                        if (objectness < ScoreThreshold) continue;
                        int classId = 0;
                        for (int id = 1; id < ClassCount; ++id)
                            if (values[5 + id] > values[5 + classId]) classId = id;
                        const float confidence = objectness * dequantize(values[5 + classId], output.attr);
                        if (confidence < ScoreThreshold) continue;
                        const float cx = (dequantize(values[0], output.attr) * 2 - 0.5f + col) * stride;
                        const float cy = (dequantize(values[1], output.attr) * 2 - 0.5f + row) * stride;
                        const float w = dequantize(values[2], output.attr) * 2;
                        const float h = dequantize(values[3], output.attr) * 2;
                        const float width = w * w * Anchors[head][anchor * 2];
                        const float height = h * h * Anchors[head][anchor * 2 + 1];
                        const float sx = static_cast<float>(rgb.cols) / resizedWidth;
                        const float sy = static_cast<float>(rgb.rows) / resizedHeight;
                        Detection box{classId, confidence,
                            std::clamp((cx - width / 2 - left) * sx, 0.0f, static_cast<float>(rgb.cols)),
                            std::clamp((cy - height / 2 - top) * sy, 0.0f, static_cast<float>(rgb.rows)),
                            std::clamp((cx + width / 2 - left) * sx, 0.0f, static_cast<float>(rgb.cols)),
                            std::clamp((cy + height / 2 - top) * sy, 0.0f, static_cast<float>(rgb.rows))};
                        if (box.x2 <= box.x1 || box.y2 <= box.y1) continue;
                        if (candidates.size() == CandidateLimit) {
                            error = "Too many detection candidates; frame rejected.";
                            return false;
                        }
                        candidates.push_back(box);
                    }
                }
            }
        }
        std::sort(candidates.begin(), candidates.end(), [](const Detection& a, const Detection& b) {
            return a.confidence > b.confidence;
        });
        for (const auto& box : candidates) {
            bool duplicate = false;
            // 一颗棋子的类型互斥，跨类别去掉不同分支给同一棋子的重复框。
            for (const auto& accepted : result)
                if (intersectionOverUnion(box, accepted) > IouThreshold) {
                    duplicate = true;
                    break;
                }
            if (!duplicate) result.push_back(box);
        }
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        result.clear();
        return false;
    }
}
