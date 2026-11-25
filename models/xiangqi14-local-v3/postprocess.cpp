// Copyright (c) 2021 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Adapted from Rockchip rknn_model_zoo @ 837db8e992815ff8647eadb709f07067f4bc3a71
// examples/yolov5/cpp/postprocess.cc (process_i8_rv1106).
// Modified for the 14-class NHWC model, padded rows, inclusive score threshold,
// stable cross-class NMS, subpixel boxes and direct-crop coordinates.
#include "postprocess.hpp"
#include "yolo_anchors.hpp"
#include <algorithm>

namespace scara
{
static float deqnt_affine_to_f32(int8_t qnt, int32_t zp, float scale) { return ((float)qnt - (float)zp) * scale; }

// Rockchip 的 RV1106 INT8 解码分支，行首使用 RKNN 返回的实际字节跨度。
static void process_i8_rv1106(const YoloHead &head, int index, std::vector<Detection> &boxes)
{
    const int grid = YoloGrid[index];
    const int stride = YoloSize / grid;
    const float threshold = 0.5;
    const int32_t zp = head.zeroPoint;
    const float scale = head.scale;

    for (int h = 0; h < grid; ++h) {
        const int8_t *row = head.data + h * head.rowBytes;
        for (int w = 0; w < grid; ++w) {
            for (int a = 0; a < YoloAnchors; ++a) {
                const int8_t *hw_ptr = row + w * YoloChannels + a * YoloBoxValues;
                int8_t maxClassProbs = hw_ptr[5];
                int maxClassId = 0;
                for (int k = 1; k < YoloClasses; ++k) {
                    int8_t prob = hw_ptr[5 + k];
                    if (prob > maxClassProbs) {
                        maxClassId = k;
                        maxClassProbs = prob;
                    }
                }

                float box_conf_f32 = deqnt_affine_to_f32(hw_ptr[4], zp, scale);
                float class_prob_f32 = deqnt_affine_to_f32(maxClassProbs, zp, scale);
                float limit_score = box_conf_f32 * class_prob_f32;
                if (limit_score < threshold) continue;

                float box_x = deqnt_affine_to_f32(hw_ptr[0], zp, scale) * 2.0 - 0.5;
                float box_y = deqnt_affine_to_f32(hw_ptr[1], zp, scale) * 2.0 - 0.5;
                float box_w = deqnt_affine_to_f32(hw_ptr[2], zp, scale) * 2.0;
                float box_h = deqnt_affine_to_f32(hw_ptr[3], zp, scale) * 2.0;
                box_w = box_w * box_w * ModelAnchors[index][a].width;
                box_h = box_h * box_h * ModelAnchors[index][a].height;
                // 保留亚像素坐标，给棋盘归格和相邻框的去重使用。
                double left = (double(box_x) + w) * stride - box_w / 2.0;
                double top = (double(box_y) + h) * stride - box_h / 2.0;
                boxes.push_back({maxClassId, limit_score, {left, top, box_w, box_h}});
            }
        }
    }
}

static bool higherScore(const Detection &a, const Detection &b)
{
    return a.confidence > b.confidence;
}

std::vector<Detection> Vision_decode(const YoloHead heads[YoloHeads], cv::Point origin)
{
    std::vector<Detection> candidates;
    for (int i = 0; i < YoloHeads; ++i)
        process_i8_rv1106(heads[i], i, candidates);
    std::stable_sort(candidates.begin(), candidates.end(), higherScore);

    // 同一枚棋子的不同类别预测也互相去重，取框顺序与既有模型回放一致。
    const float overlapThreshold = 0.45;
    cv::Rect2d crop(origin.x, origin.y, YoloSize, YoloSize);
    std::vector<Detection> pieces;
    for (size_t i = 0; i < candidates.size(); ++i) {
        Detection &piece = candidates[i];
        if (piece.bbox.empty()) continue;
        for (size_t j = i + 1; j < candidates.size(); ++j) {
            cv::Rect2d &other = candidates[j].bbox;
            if (1.0 - cv::jaccardDistance(piece.bbox, other) > overlapThreshold) other = {};
        }
        piece.bbox = (piece.bbox + cv::Point2d(origin)) & crop;
        if (!piece.bbox.empty()) pieces.push_back(piece);
    }
    return pieces;
}
} // namespace scara
