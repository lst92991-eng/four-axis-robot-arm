// 复用六轴已验证的基本标定器；这里只适配本项目电流单位与三环入口。
#include "calibration.h"
#include "motor_control.h"
#include "motion.h"
#include "current_limits.h"
#include "at32m412_416_conf.h"
#include <limits.h>

#ifndef SLOT_A
#define SLOT_A 0x0801F000UL
#define SLOT_B 0x0801F400UL
#endif
static motor_cal_t cal;
static const motor_cal_record_t *saved;
static const motor_cal_record_t *runtime;

static const motor_cal_record_t *load(void)
{
  const motor_cal_record_t *a=(const motor_cal_record_t *)SLOT_A;
  const motor_cal_record_t *b=(const motor_cal_record_t *)SLOT_B;
  int av=motor_cal_record_valid(a) && a->pole_pairs==CONFIG_ENCODER_POLES;
  int bv=motor_cal_record_valid(b) && b->pole_pairs==CONFIG_ENCODER_POLES;
  if(!av)return bv?b:0;
  return bv && (int32_t)(b->sequence-a->sequence)>0?b:a;
}
void Calibration_init(void)
{
  motor_cal_init(&cal);
  runtime=saved=load();
}
int Calibration_active(void){return motor_cal_active(&cal);}
int Calibration_ready(void){return runtime!=0;}
int Calibration_stored(void){return saved!=0;}
int Calibration_direction(void){return runtime?runtime->direction:CONFIG_ENCODER_DIR;}
const motor_cal_t *Calibration_status(void){return &cal;}
const motor_cal_record_t *Calibration_record(void){return runtime;}
float Calibration_angle(uint16_t raw)
{
  if(runtime)return motor_cal_angle(runtime,raw)*(6.28318530718f/360000.0f);
  int32_t phase=((int32_t)raw-CONFIG_ENCODER_ZERO)*CONFIG_ENCODER_DIR*CONFIG_ENCODER_POLES%16384;
  return phase*(6.28318530718f/16384);
}
void Calibration_abort(motor_cal_error_t reason)
{
  if(!Calibration_active())return;
  // 主循环和命令入口调用；硬件 ISR 故障仍由 Motor_shutdown 独立关断。
  Motor_calibrationDrive(0,0,0);
  motor_cal_abort(&cal,reason);
}
static motor_cal_input_t input(const motor_status_t *b,uint32_t now)
{
  motor_cal_input_t in={0};
  in.now_ms=now;
  in.counts=(int32_t)b->encoder_counts;
  in.phase_a=b->phase_a_counts;
  in.phase_b=b->phase_b_counts;
  in.id_actual=b->id_counts;
  in.bus_mv=b->bus_mv;
  in.sensors_ready=b->encoder_valid && b->encoder_counts>=INT32_MIN+1LL && b->encoder_counts<=INT32_MAX-1LL;
  if(cal.state==MOTOR_CAL_PRECHECK){in.phase_a=b->phase_a_mean;in.phase_b=b->phase_b_mean;}
  return in;
}
int Calibration_command(int op,int current_ma,const motor_status_t *b,uint32_t now)
{
  if(op==0)return 1;
  if(op==2){Calibration_abort(MOTOR_CAL_USER);return 1;}
  if(CONFIG_NODE!=1)return 0; // 本次只允许 J1 基本标定，不意外扩到其他轴。
  if(Joint.status.state!=MOTION_OFF || b->power || b->fault || Calibration_active())return 0;
  if(op==1)
  {
    if(current_ma<100 || current_ma>1000 || current_ma>CONFIG_CURRENT_MA)return 0;
    int32_t counts=(int32_t)((int64_t)current_ma*4095*10*20/(3300*1000));
    motor_cal_input_t in=input(b,now);
    in.phase_a=b->phase_a_mean;in.phase_b=b->phase_b_mean;
    if(runtime){in.alignment_hint_valid=1;in.alignment_angle_mdeg=motor_cal_angle(runtime,(uint16_t)b->encoder_counts);}
    // ISR 即使未出力也读取电角映射；先撤走 RAM 候选指针，再清空候选。
    const motor_cal_record_t *prior=runtime;
    runtime=saved;
    if(!motor_cal_start(&cal,CONFIG_ENCODER_POLES,(uint16_t)counts,0,&in)){runtime=prior;return 0;}
    motor_configured=(uint8_t)(runtime!=0 || (CONFIG_ENCODER_ZERO>=0 && CONFIG_ENCODER_ZERO<16384));
    Joint.status.referenced=0;
    return 1;
  }
  if(op!=3 || cal.state!=MOTOR_CAL_DONE || !motor_cal_record_valid(&cal.candidate))return 0;
  if(saved && saved->crc==cal.candidate.crc)return 1;
  uintptr_t address=saved==(const motor_cal_record_t *)SLOT_A?SLOT_B:SLOT_A;
  motor_cal_seal(&cal.candidate,saved?saved->sequence+1U:1U);
  const uint32_t *words=(const uint32_t *)&cal.candidate;
  uint32_t irq=__get_PRIMASK();
  __disable_irq();
  flash_unlock();flash_flag_clear(FLASH_PRGMERR_FLAG|FLASH_EPPERR_FLAG);
  flash_status_type result=flash_sector_erase(address);
  for(uint32_t i=0;result==FLASH_OPERATE_DONE && i<sizeof(cal.candidate)/4U;i++)
    result=flash_word_program(address+i*4U,words[i]);
  flash_lock();__set_PRIMASK(irq);
  if(result!=FLASH_OPERATE_DONE || !motor_cal_record_valid((const motor_cal_record_t *)address))return 0;
  runtime=saved=load();
  return saved!=0;
}
void Calibration_process(const motor_status_t *b,uint32_t now)
{
  if(!Calibration_active())return;
  if(b->fault){Calibration_abort(b->fault==FAULT_CURRENT?MOTOR_CAL_CURRENT:MOTOR_CAL_SENSOR);return;}
  motor_cal_input_t in=input(b,now);
  motor_cal_step(&cal,&in);
  if(!Calibration_active())
  {
    Motor_calibrationDrive(0,0,0);
    if(cal.state==MOTOR_CAL_DONE && motor_cal_record_valid(&cal.candidate))
    {
      runtime=&cal.candidate;
      motor_configured=1;
    }
    return;
  }
  if(!Motor_calibrationDrive(cal.drive,cal.drive_angle,current_counts_to_ma(cal.drive_reference)))
    Calibration_abort(MOTOR_CAL_SENSOR);
}
