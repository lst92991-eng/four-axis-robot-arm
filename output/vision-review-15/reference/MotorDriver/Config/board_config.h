#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* 第七章电气采样参数；单位写入宏名，换算系数由电路比例与实测校准确定。 */
#define BOARD_ADC_PERIOD_MS          20U
#define BOARD_ADC_FULL_SCALE       4095U
#define BOARD_BUS_UV_PER_COUNT     8850UL
#define BOARD_BUS_OFFSET_MV           0L
#define BOARD_CURRENT_UA_PER_COUNT  806UL
#define BOARD_BUS_UV_TRIP_MV      18000UL
#define BOARD_BUS_UV_RECOVER_MV   19200UL
#define BOARD_BUS_OV_TRIP_MV      25800UL
#define BOARD_BUS_OV_RECOVER_MV   25200UL
/* 任一侧电流达到800mA并持续5个名义采样周期，即约100ms，锁存故障。 */
#define BOARD_PEAK_CURRENT_MA       800U
#define BOARD_PEAK_CURRENT_TICKS      5U
#define BOARD_ADC_STALE_MS           60U
#define BOARD_ADC_ZERO_SAMPLES       20U
/* 只允许接近零电流的样本参与校准，避免把真实负载电流扣掉。 */
#define BOARD_ADC_ZERO_MAX_RAW       19U

/* 沿用完整电机板的CAN FD参数和四种消息。 */
#define BOARD_CONTROL_PERIOD_MS       10U
#define BOARD_SPEED_PERIOD_MS         20U
#define BOARD_STATUS_PERIOD_MS       100U
#define BOARD_IMU_STALE_MS            50U
#define BOARD_CAN_ID_CONTROL       0x201U
#define BOARD_CAN_ID_REFERENCE     0x401U
#define BOARD_CAN_ID_STATUS        0x181U
#define BOARD_CAN_ID_ODOMETRY      0x381U
#define BOARD_CAN_REFERENCE_SIZE      16U
#define BOARD_MAX_LINEAR_MM_S        300U
#define BOARD_MAX_ANGULAR_MRAD_S     1200U

#endif
