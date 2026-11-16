#ifndef MOTOR_LESSON_CONFIG_H
#define MOTOR_LESSON_CONFIG_H

/* 本章短时架空实验；不是车体速度目标。 */
#define LESSON_PWM_PERMILLE 600
#define LESSON_RUN_MS      5000U
#define LESSON_SAMPLE_MS     20U
#define LESSON_MAX_RPM      75.0f
/* 11对磁极、60:1减速比、AB四倍频：11*60*4，按实际到货规格核对。 */
#define LESSON_COUNTS_PER_REV 2640.0f
/* PID初值来自参考参数，仍需在具体电机上调参。 */
#define LESSON_LEFT_PID_KP          4.0f
#define LESSON_LEFT_PID_KI          8.0f
#define LESSON_LEFT_PID_KD          0.0f
#define LESSON_RIGHT_PID_KP         4.0f
#define LESSON_RIGHT_PID_KI         8.0f
#define LESSON_RIGHT_PID_KD         0.0f
#define LESSON_PID_I_LIMIT        250.0f
#define LESSON_FF_STATIC          100.0f
#define LESSON_FF_PER_RPM           9.4f
#define LESSON_RPM_FILTER_ALPHA      0.20f
#define LESSON_RAMP_RPM_S          60.0f
#define LESSON_TEST_RPM            30.0f
/* 几何参数为参考值，未装整车时只用于计算演示。 */
#define LESSON_WHEEL_CIRCUMFERENCE_MM 267.035f
#define LESSON_TRACK_MM           500.0f
/* 把课件原始计数与驱动方向都对应到车体前进为正；按第六章记录核对。 */
#define LESSON_LEFT_OUTPUT_SIGN    -1.0f
#define LESSON_RIGHT_OUTPUT_SIGN    1.0f
#define LESSON_LEFT_FEEDBACK_SIGN   1.0f
#define LESSON_RIGHT_FEEDBACK_SIGN  1.0f
/* 板上Z轴与车体向上同向时为+1；翻转安装时重新核对。 */
#define LESSON_IMU_Z_SIGN             1.0f
/* 单板阶段为0：分别观察轮式里程与陀螺积分。装车并核对坐标后才设1。 */
#define LESSON_ODOM_FUSION            0
#define LESSON_ODOM_TAU_S             2.0f
/* 仅控制板和电机装为同一车体后开启，单板分开放置保持0。 */
#define LESSON_BODY_IMU_CONTROL       0
#define LESSON_YAW_RATE_GAIN          0.35f
#endif
