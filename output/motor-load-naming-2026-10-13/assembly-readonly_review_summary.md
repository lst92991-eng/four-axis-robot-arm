# SolidWorks Review Summary

- Status: `warn`
- Score: `64`
- Manual review required: `True`

## Checks

- `rebuild_succeeded`: `True`
- `model_available`: `True`
- `previews_created`: `True`
- `previews_not_blank`: `True`
- `expected_outputs_exist`: `None`
- `feature_summary_available`: `True`
- `geometry_measurements_available`: `False`
- `geometry_measurements_error_free`: `True`
- `feature_errors_absent`: `True`

## Issues

- `warn` `feature_health_review_required`: 特征存在警告或无法取得完整错误证据。
- `warn` `interference_detected`: 装配体存在干涉，体积与组件详见明细。
- `warn` `expected_outputs_not_declared`: 未声明期望输出文件，无法验证交付物是否完整。

## Recommendations

- 复核特征错误/警告明细。
- 确认是否为有意过盈，否则修复配合。
- 调用 run_review() 时传入 expected_outputs。

## Previews

- `C:\Users\lst\Desktop\scara\output\motor-load-naming-2026-10-13\assembly-readonly_isometric.bmp`
- `C:\Users\lst\Desktop\scara\output\motor-load-naming-2026-10-13\assembly-readonly_front.bmp`
- `C:\Users\lst\Desktop\scara\output\motor-load-naming-2026-10-13\assembly-readonly_top.bmp`
- `C:\Users\lst\Desktop\scara\output\motor-load-naming-2026-10-13\assembly-readonly_right.bmp`
