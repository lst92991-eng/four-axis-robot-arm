#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H
#include <stdint.h>

//---------节点与电角配置区---------
//can地址
#define CONFIG_NODE 1
//电角度零点，-1禁止使能
#define CONFIG_ENCODER_ZERO (-1) 
//电子极对数
#define CONFIG_ENCODER_POLES 50
//编码器方向，1或-1
#define CONFIG_ENCODER_DIR 1
//---------机械配置区---------
//减速比
#define CONFIG_RATIO 20.0f
//关节方向
#define CONFIG_DIRECTION 1
//最小角度和最大角度
#define CONFIG_MINIMUM (-180.0f)
#define CONFIG_MAXIMUM 180.0f
//---------运动限制区---------
//速度和加速度
#define CONFIG_VELOCITY 10.0f
#define CONFIG_ACCELERATION 30.0f
//允许速度环请求的最大电流
#define CONFIG_CURRENT_MA 1000.0f
//---------控制参数区---------
//速度环的PID参数
#define CONFIG_SPEED_KP 20.0f
#define CONFIG_SPEED_KI 150.0f
//位置环的PID参数
#define CONFIG_POSITION_KP 8.0f
//电流环的PID参数
#define CONFIG_CURRENT_KP 3.0f
#define CONFIG_CURRENT_KI 0.7f
//---------电压限幅区---------
//电压矢量限制，L2范数，单位mV
#define CONFIG_VOLTAGE_MV 6000
#endif
