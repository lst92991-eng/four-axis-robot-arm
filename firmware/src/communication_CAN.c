//接收命令、执行对应动作、返回应答和运行状态
#include "communication_CAN.h"
#include "CAN.h"
#include "motor_control.h"
#include "calibration.h"
#include "current_limits.h"
#include "at32m412_416_conf.h"
#include "build_id.h"
#include <math.h>
#include <string.h>

//---------通信状态区---------
static struct
{
  //防止重复执行的命令序号
  uint32_t sequence;
  //上一次接受有效命令的时刻，毫秒
  uint32_t last_command_ms;
  //上一次发送状态的时刻，毫秒
  uint32_t last_status_ms;
  //CAN硬件是否初始化成功
  uint8_t initialized;
  uint8_t reset_pending;
  uint32_t reset_at;
  uint8_t reply_pending;
  uint8_t reply[64];
} Protocol;

//---------内部声明区---------
static int processFrame(const motor_status_t *motor, uint32_t now,
                        const uint8_t request[64], uint8_t response[64]);
static void packStatus(const motor_status_t *motor, uint32_t now, uint8_t response[64]);

//---------初始化区---------
//先检查节点地址，再记录CAN硬件初始化结果
void CAN_protocol_setup(void)
{
  Protocol.sequence = 0;
  Protocol.last_command_ms = 0;
  Protocol.last_status_ms = 0;
  Protocol.initialized = 0;
  Protocol.reset_pending = 0;
  Protocol.reply_pending = 0;
  if (joint_config->can_node < 1 || joint_config->can_node > 127)
    return;
  Protocol.initialized = CAN_init();
}

//---------通信处理区---------
//先接收一帧并回应，再按50ms周期发送运行状态
void CAN_protocol_process(const motor_status_t *motor, uint32_t now)
{
  if (!Protocol.initialized)
    return;
  if(Protocol.reset_pending && (int32_t)(now-Protocol.reset_at)>=0)
    NVIC_SystemReset();
  if(Protocol.reply_pending && CAN_send((uint16_t)(0x480+joint_config->can_node),Protocol.reply))
    Protocol.reply_pending=0;
  //完整帧才会写入有效ID，0表示本次没有可处理的命令
  uint16_t id = 0;
  uint8_t data[64];
  if(!Protocol.reply_pending)CAN_receive(&id, data);
  if (id == 0x400 + joint_config->can_node)
  {
    if (processFrame(motor, now, data, Protocol.reply))
    {
      Protocol.reply_pending=1;
      if(CAN_send((uint16_t)(0x480+joint_config->can_node),Protocol.reply))Protocol.reply_pending=0;
    }
  }
  //状态发送到期后组帧，成功写入发送队列后更新时刻
  if (now - Protocol.last_status_ms < 50)
    return;
  packStatus(motor, now, data);
  if (CAN_send((uint16_t)(0x200 + joint_config->can_node), data))
    Protocol.last_status_ms = now;
}

//超过500ms没有有效命令时请求减速保持
void CAN_protocol_watchdog(uint32_t now)
{
  const motion_t *s = &Joint.status;
  if (now - Protocol.last_command_ms <= 500)
    return;
  if (Calibration_active()) Calibration_abort(MOTOR_CAL_LINK);
  if (s->interlocked)
    return;
  //停止入口统一检查当前是否处于闭环状态
  Motion_stop(1);
}

//---------回调区---------
//命令回调返回1表示接受，返回0表示拒绝
static int handleZero(const CanPacket *command, const motor_status_t *feedback)
{
  if (Calibration_active()) return 0;
  if (feedback->power || !feedback->encoder_valid)
    return 0;
  if (!motor_configured)
    return 0;
  return Motion_setZero(feedback->encoder_counts, command->values[0] / 1000.0f);
}

static int handleMove(const CanPacket *command, const motor_status_t *feedback)
{
  (void)feedback;
  if (!Motor_canRun())
    return 0;
  float target = command->values[0] / 1000.0f;
  //相对增量只换算一次，随后统一处理绝对目标
  if (command->kind == CMD_MOVE_REL)
    target += Joint.status.position;
  float speed = command->values[1] ? command->values[1] / 1000.0f : joint_config->default_speed_deg_s;
  return Motion_moveTo(target, speed);
}

static int handleStop(const CanPacket *command, const motor_status_t *feedback)
{
  (void)command;
  (void)feedback;
  if (Calibration_active()){Calibration_abort(MOTOR_CAL_USER);return 1;}
  return Motion_stop(0);
}

static int handleOff(const CanPacket *command, const motor_status_t *feedback)
{
  (void)command;
  (void)feedback;
  Calibration_abort(MOTOR_CAL_USER);
  Motion_release();
  return 1;
}

static int handleClear(const CanPacket *command, const motor_status_t *feedback)
{
  if (Calibration_active()) return 0;
  (void)command;
  if (Joint.status.interlocked && Joint.status.state == MOTION_HOLD &&
      Joint.status.stationary && Joint.status.referenced && feedback->encoder_valid && !feedback->fault)
  {
    Joint.status.interlocked = 0;
    return 1;
  }
  if (Joint.status.state != MOTION_FAULT) return 0;
  int accepted = Motor_clearFault();
  if (accepted)
    Motion_clearFault();
  return accepted;
}

static int handlePing(const CanPacket *command, const motor_status_t *feedback)
{
  (void)command;
  (void)feedback;
  return 1;
}

static int handleCalibrate(const CanPacket *command,const motor_status_t *feedback)
{
  return Calibration_command(command->values[0],command->values[1],feedback,system_ms);
}

static int handleTune(const CanPacket *command,const motor_status_t *feedback)
{
  if(feedback->power || feedback->fault || !feedback->encoder_valid)return 0;
  return Motor_setSpeedGains(command->values[0],command->values[1]);
}
static int handleReboot(const CanPacket *command,const motor_status_t *feedback)
{
  if(joint_config->can_node!=1 || command->values[0]!=0x52454254 || feedback->power ||
     Joint.status.state!=MOTION_OFF || Calibration_active())return 0;
  Protocol.reset_pending=1;Protocol.reset_at=system_ms+100;
  return 1;
}
static int handleVelocity(const CanPacket *command,const motor_status_t *feedback)
{
  (void)feedback;
  return Motor_canRun()?Motion_runVelocity(command->values[0]/1000.0f):0;
}
static int handleBrakeRate(const CanPacket *command,const motor_status_t *feedback)
{
  if(feedback->fault || !feedback->encoder_valid || Calibration_active())return 0;
  return Motion_setBrakeAcceleration(command->values[0]/1000.0f);
}

//---------命令映射区---------
//按命令编号选择处理任务，绝对和相对运动共用回调
typedef int (*command_handler_t)(const CanPacket *, const motor_status_t *);
static const command_handler_t command_handlers[] =
{
  [CMD_ZERO] = handleZero,
  [CMD_MOVE] = handleMove,
  [CMD_MOVE_REL] = handleMove,
  [CMD_STOP] = handleStop,
  [CMD_OFF] = handleOff,
  [CMD_CLEAR] = handleClear,
  [CMD_PING] = handlePing,
  [CMD_CALIBRATE] = handleCalibrate,
  [CMD_TUNE_SPEED] = handleTune,
  [CMD_REBOOT] = handleReboot,
  [CMD_VELOCITY] = handleVelocity,
  [CMD_BRAKE_RATE] = handleBrakeRate
};

//---------报文解析区---------
//返回1表示已生成应答，返回0表示忽略此帧
static int processFrame(const motor_status_t *feedback, uint32_t now, const uint8_t request[64],
                        uint8_t response[64])
{
  //读取本节点的v3请求，结果字段由应答方填写
  CanPacket command;
  memcpy(&command, request, sizeof(command));
  if (command.version != 3)
    return 0;
  if (command.node != joint_config->can_node)
    return 0;
  if (command.result)
    return 0;

  //失能设零允许重建序号，同一序号仍拒绝重复执行
  int accepted = 0;
  int stale = (int32_t)(command.sequence - Protocol.sequence) <= 0;
  if (command.kind == CMD_ZERO && Joint.status.state == MOTION_OFF)
    stale = command.sequence == Protocol.sequence;
  //新请求和失能命令进入分派，其余请求统一拒绝
  if (!stale || command.kind == CMD_OFF)
  {
    //普通命令先占用序号，执行被拒绝后也不重复执行
    if (command.kind != CMD_ZERO && command.kind != CMD_OFF)
      Protocol.sequence = command.sequence;
    //先检查表范围，再检查该编号是否配置了处理任务
    if (command.kind < sizeof(command_handlers) / sizeof(command_handlers[0]))
    {
      command_handler_t handle = command_handlers[command.kind];
      if (handle)
        accepted = handle(&command, feedback);
    }
  }

  //成功命令刷新通信时间，设零在成功后提交序号；失能不改这两项
  if (accepted && command.kind != CMD_OFF)
  {
    Protocol.sequence = command.sequence;
    Protocol.last_command_ms = now;
  }

  //统一应答：0接受，1拒绝
  CanPacket reply = {0};
  reply.kind = command.kind | 0x80;
  reply.node = joint_config->can_node;
  reply.version = 3;
  reply.sequence = command.sequence;
  reply.result = !accepted;
  if(accepted && command.kind==CMD_TUNE_SPEED)Motor_readSpeedGains(reply.values);
  if(accepted && command.kind==CMD_BRAKE_RATE)reply.values[0]=(int32_t)lroundf(Motion_brakeAcceleration()*1000);
  memcpy(response, &reply, sizeof(reply));
  return 1;
}

//---------状态反馈区---------
//把位置、速度和运行状态写入固定报文
static void packStatus(const motor_status_t *b, uint32_t now, uint8_t response[64])
{
  const motion_t *s = &Joint.status;
  CanPacket r = {0};
  r.kind = MSG_STATUS;
  r.node = joint_config->can_node;
  r.version = 3;
  r.values[0] = (int32_t)now;
  r.values[1] = (int32_t)lroundf(s->position * 1000);
  r.values[2] = (int32_t)lroundf(s->requested * 1000);
  r.values[4] = (int32_t)lroundf(s->velocity * 1000);
  uint32_t flags = 1U | s->referenced << 1 | s->arrived << 2 | s->interlocked << 3 |
                   motor_configured << 4 | b->encoder_valid << 5 | b->power << 6 | s->stationary << 8;
  r.values[9] = (int32_t)((uint32_t)s->state | ((uint32_t)s->fault << 8) | (flags << 16));
  r.values[10] = (int32_t)JOINT_BUILD_ID;
  const motor_cal_t *cal=Calibration_status();
  const motor_cal_record_t *record=Calibration_record();
  r.values[3]=(int32_t)cal->state | ((int32_t)cal->error<<8) |
              (Calibration_ready()<<16) | (Calibration_stored()<<17) |
              ((int32_t)s->fault_detail<<24);
  r.values[5]=(int32_t)b->encoder_counts;
  r.values[6]=record?record->raw_zero:-1;
  r.values[7]=record?record->direction:0;
  r.values[8]=current_counts_to_ma(cal->current);
  r.values[11]=cal->progress;
  r.values[12]=(int32_t)b->bus_mv;
  r.values[13]=0x43414c31;
  memcpy(response, &r, sizeof(r));
}
