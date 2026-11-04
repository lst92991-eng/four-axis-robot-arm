//按电流反馈、位置环、速度环、电流环的顺序计算
/*
 * Copyright (c) Source robotics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 */
/* 控制公式源自STEPFOC，按GPL-3.0-or-later管理；本项目整理输入输出与计算步骤。
 * 固定来源和迁移记录见upstream.lock.json，协议与硬件资格在计算函数之外处理。 */
#include "Common_Control.h"
#include "dsp/fast_math_functions.h"
#include <math.h>

//---------限幅区---------
static float limit_value(float value, float minimum, float maximum)
{
  if (value < minimum)
    return minimum;
  if (value > maximum)
    return maximum;
  return value;
}

static void limit_vector(float *d, float *q, float limit)
{
  float length = sqrtf(*d * *d + *q * *q);
  if (length > limit)
  {
    float ratio = limit / length;
    *d = *d * ratio;
    *q = *q * ratio;
  }
}

//---------坐标变换区---------
//把两相电流换算为d轴和q轴反馈
void Common_Park_Compute(Current_Struct *current, int32_t a, int32_t b, float angle)
{
  float sine = arm_sin_f32(angle);
  float cosine = arm_cos_f32(angle);
  current->d.currentValue = a * cosine + b * sine;
  current->q.currentValue = b * cosine - a * sine;
}

//---------位置环区---------
//位置误差经过比例计算和限幅，得到速度给定
void Common_Position_Compute(Position_Struct *position)
{
  float error = position->targetValue - position->currentValue;
  float p = position->KP * error;
  position->out = limit_value(p, -position->limit, position->limit);
}

//---------速度环区---------
//速度误差经过PI计算和限幅，得到q轴电流给定
void Common_Speed_Compute(Speed_Struct *speed)
{
  float error = speed->targetValue - speed->currentValue;
  float oldSum = speed->sumError;

  speed->sumError += error * speed->KI;
  speed->sumError = limit_value(speed->sumError, -speed->limit, speed->limit);
  float piOut = speed->KP * error + speed->sumError;
  speed->out = limit_value(piOut, -speed->limit, speed->limit);

  //限流后停止继续积累同方向的误差
  int satHigh = speed->out >= speed->limit && error > 0;
  int satLow = speed->out <= -speed->limit && error < 0;
  if (satHigh || satLow)
    speed->sumError = oldSum;
}

//---------电流环区---------
//电流误差经过PI和逆Park变换，得到两相电压
void Common_Current_Compute(Current_Struct *current, float angle, int32_t bus)
{
  float errD = current->d.targetValue - current->d.currentValue;
  float errQ = current->q.targetValue - current->q.currentValue;
  current->d.sumError += errD * current->d.KI;
  current->q.sumError += errQ * current->q.KI;

  float voltageLimit = current->voltageLimit;
  if (current->voltageLimit <= 0 || current->voltageLimit > bus)
    voltageLimit = bus;
  limit_vector(&current->d.sumError, &current->q.sumError, voltageLimit);

  float pD = current->d.KP * errD;
  float pQ = current->q.KP * errQ;
  current->d.out = pD + current->d.sumError;
  current->q.out = pQ + current->q.sumError;
  limit_vector(&current->d.out, &current->q.out, voltageLimit);

  float sine = arm_sin_f32(angle);
  float cosine = arm_cos_f32(angle);
  float phaseA = current->d.out * cosine - current->q.out * sine;
  float phaseB = current->q.out * cosine + current->d.out * sine;
  current->outA = limit_value(phaseA, -bus, bus);
  current->outB = limit_value(phaseB, -bus, bus);
}
