# 从随 v3 模型发布的清单取得参考框。每路六个数，依次为三对宽、高。
set(model_info "${CMAKE_CURRENT_SOURCE_DIR}/models/xiangqi14-local-v3/model-info.json")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${model_info}")
file(READ "${model_info}" model_json)
set(anchor_rows "")
foreach(head RANGE 0 2)
  string(JSON sizes GET "${model_json}" anchors ${head})
  string(APPEND anchor_rows "    {")
  foreach(w IN ITEMS 0 2 4)
    math(EXPR h "${w} + 1")
    string(JSON width GET "${sizes}" ${w})
    string(JSON height GET "${sizes}" ${h})
    string(APPEND anchor_rows "{${width}, ${height}}, ")
  endforeach()
  string(APPEND anchor_rows "},\n")
endforeach()
file(CONFIGURE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/yolo_anchors.hpp" CONTENT
"#pragma once
#include \"models/xiangqi14-local-v3/postprocess.hpp\"
namespace scara {
inline const cv::Size2f ModelAnchors[YoloHeads][YoloAnchors] = {
@anchor_rows@};
}
" @ONLY)
