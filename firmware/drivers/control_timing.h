/** @file control_timing.h
 * @brief 统一CPU、PWM、ADC窗口及三环控制的时序常量。 */
#ifndef CONTROL_TIMING_H
#define CONTROL_TIMING_H
//---------时基参数区---------
/* AT32M416时序约定：主频180MHz，PWM/ADC采样25kHz，三环控制6250Hz。 */
#define CONTROL_CORE_CLOCK_HZ             180000000UL
#define CONTROL_PWM_FREQUENCY_HZ              25000UL
#define CONTROL_PWM_TIMER_PERIOD               7199UL
#define CONTROL_PWM_DEADTIME_TICKS                90UL
#define CONTROL_ADC_BLANKING_TICKS               240UL
#define CONTROL_ADC_MARGIN_TICKS                 400UL

//---------采样几何约束区---------
/* 固定板级定时器参数必须留出居中PWM及其低侧采样窗口。 */
#if CONTROL_PWM_TIMER_PERIOD < 3UL || CONTROL_PWM_TIMER_PERIOD > 65535UL || \
    CONTROL_ADC_MARGIN_TICKS >= CONTROL_PWM_TIMER_PERIOD
#error "Invalid PWM sampling geometry"
#endif
#if CONTROL_ADC_BLANKING_TICKS >= CONTROL_PWM_TIMER_PERIOD - CONTROL_ADC_MARGIN_TICKS || \
    CONTROL_PWM_TIMER_PERIOD - CONTROL_ADC_MARGIN_TICKS - CONTROL_ADC_BLANKING_TICKS <= \
        (CONTROL_PWM_TIMER_PERIOD + 1UL) / 2UL
#error "Invalid PWM sampling geometry"
#endif

#define CONTROL_RUNTIME_ADC_TRIGGER_TICKS \
  (CONTROL_PWM_TIMER_PERIOD - CONTROL_ADC_MARGIN_TICKS)
/* PWM指令幅值沿用居中构帧的几何采样窗口约束。 */
#define CONTROL_PWM_SAMPLE_SAFE_COMMAND \
  (2UL * (CONTROL_RUNTIME_ADC_TRIGGER_TICKS - CONTROL_ADC_BLANKING_TICKS - \
          (CONTROL_PWM_TIMER_PERIOD + 1UL) / 2UL))
#if CONTROL_PWM_SAMPLE_SAFE_COMMAND > CONTROL_PWM_TIMER_PERIOD
#error "PWM command exceeds the fixed timer range"
#endif
//---------控制周期区---------
#define CONTROL_CURRENT_LOOP_FREQUENCY_HZ       6250UL

#define CONTROL_PWM_TIMER_COUNTS (CONTROL_PWM_TIMER_PERIOD + 1UL)

//---------时序一致性检查区---------
#if (CONTROL_CORE_CLOCK_HZ / CONTROL_PWM_TIMER_COUNTS) != \
    CONTROL_PWM_FREQUENCY_HZ
#error "PWM timing contract is inconsistent"
#endif

#if CONTROL_PWM_FREQUENCY_HZ != 4 * CONTROL_CURRENT_LOOP_FREQUENCY_HZ
#error "STEPFOC uses one 6250Hz control tick per four PWM samples"
#endif

#if CONTROL_PWM_DEADTIME_TICKS > 127UL
#error "PWM dead-time must stay in the linear timer encoding range"
#endif

#endif
