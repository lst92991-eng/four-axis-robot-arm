#include "calibration.h"
#include "motion.h"
#include "motor_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static uint32_t slots[2][256];
#define SLOT_A ((uintptr_t)slots[0])
#define SLOT_B ((uintptr_t)slots[1])
#include "../firmware/src/calibration.c"

motion_control_t Joint;
static const JointConfig profile = {
  .can_node = 1, .pole_pairs = 50, .encoder_direction = 1,
  .encoder_zero = -1, .current_limit_ma = 1000,
};
const JointConfig *joint_config = &profile;
uint8_t motor_configured;
static int driven,drive_calls,erase_calls;
static int32_t field_angle,field_ma;
uint32_t __get_PRIMASK(void){return 0;}
void __disable_irq(void){}
void __set_PRIMASK(uint32_t mask){(void)mask;}
void flash_unlock(void){assert(!driven);}
void flash_lock(void){}
void flash_flag_clear(uint32_t flags){(void)flags;}
flash_status_type flash_sector_erase(uintptr_t address)
{
  assert(!driven && (address==SLOT_A || address==SLOT_B));
  memset((void *)address,255,1024);erase_calls++;return FLASH_OPERATE_DONE;
}
flash_status_type flash_word_program(uintptr_t address,uint32_t word)
{
  assert(!driven && address>=SLOT_A && address+4<=SLOT_B+1024);
  memcpy((void *)address,&word,4);return FLASH_OPERATE_DONE;
}
int Motor_calibrationDrive(int enabled,int32_t angle,int32_t ma)
{
  driven=enabled;field_angle=angle;field_ma=ma;drive_calls++;
  assert(ma>=0 && ma<=1000);return 1;
}

int main(void)
{
  memset(slots,255,sizeof(slots));Calibration_init();
  assert(!Calibration_ready() && !Calibration_stored());
  motor_status_t b={0};b.encoder_valid=1;b.encoder_counts=12000;b.bus_mv=24000;
  assert(!Calibration_command(1,0,&b,100));
  assert(!Calibration_command(1,1001,&b,100));
  b.power=1;assert(!Calibration_command(1,300,&b,100));b.power=0;
  Joint.status.state=MOTION_HOLD;assert(!Calibration_command(1,300,&b,100));
  Joint.status.state=MOTION_OFF;
  assert(Calibration_command(1,300,&b,100));assert(!driven && drive_calls==0);
  assert(!Calibration_command(3,0,&b,100));
  int32_t previous=0;int64_t field=0;
  for(uint32_t now=101;now<15000 && Calibration_active();now++)
  {
    int32_t delta=field_angle-previous;
    if(delta>180000)delta-=360000;
    if(delta< -180000)delta+=360000;
    field+=delta;previous=field_angle;
    b.encoder_counts=12000+(int32_t)llround(field*16384.0/(50*360000.0));
    b.id_counts=(int32_t)(field_ma*819.0/3300.0);
    b.power=(uint8_t)driven;
    Calibration_process(&b,now);
  }
  assert(Calibration_status()->state==MOTOR_CAL_DONE && !driven);
  assert(Calibration_ready() && !Calibration_stored() && motor_configured);
  assert(Calibration_record()->raw_zero==12000 && Calibration_direction()==1);
  b.power=0;
  assert(Calibration_command(3,0,&b,15000));assert(erase_calls==1);
  assert(Calibration_command(3,0,&b,15001));assert(erase_calls==1);
  Calibration_init();assert(Calibration_ready() && Calibration_stored());
  assert(Calibration_record()->raw_zero==12000);
  assert(Calibration_command(1,300,&b,16000));
  b.fault=FAULT_SENSOR;Calibration_process(&b,16001);
  assert(!Calibration_active() && !driven && Calibration_status()->error==MOTOR_CAL_SENSOR);
  assert(Calibration_stored());
  puts("calibration_adapter: PASS (gates, units, ramp, save/reload, abort)");
  return 0;
}
