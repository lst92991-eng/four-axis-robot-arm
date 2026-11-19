#include "motor_calibration.h"
#include "current_limits.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void){
 for(int hint=0;hint<360000;hint+=17000){
  motor_cal_t c;motor_cal_input_t in;memset(&in,0,sizeof in);motor_cal_init(&c);
  in.sensors_ready=1;in.bus_mv=24000;in.now_ms=100;in.counts=12000;
  in.alignment_hint_valid=1;in.alignment_angle_mdeg=(unsigned)hint;
  assert(motor_cal_start(&c,50,CURRENT_CAL_REFERENCE_DEFAULT,1,&in));
  assert(!c.drive && c.drive_angle==hint && c.drive_target==hint);
  while(c.state==MOTOR_CAL_PRECHECK){in.now_ms++;motor_cal_step(&c,&in);}
  assert(c.state==MOTOR_CAL_ALIGN && c.drive && c.drive_angle==hint && c.drive_target==0);
  for(int n=0;n<400;n++){
   int prior=c.drive_angle;in.now_ms++;in.id_actual=c.drive_reference;motor_cal_step(&c,&in);
   int delta=c.drive_angle-prior;if(delta>180000)delta-=360000;if(delta< -180000)delta+=360000;
   assert(delta>=-500 && delta<=500);
   assert(c.state==MOTOR_CAL_ALIGN);
  }
  assert(c.drive_angle==0);
 }
 motor_cal_t c;motor_cal_input_t in;memset(&in,0,sizeof in);motor_cal_init(&c);
 in.sensors_ready=1;in.bus_mv=24000;in.alignment_hint_valid=1;in.alignment_angle_mdeg=360000;
 assert(!motor_cal_start(&c,50,CURRENT_CAL_REFERENCE_DEFAULT,1,&in));
 in.alignment_hint_valid=0;assert(motor_cal_start(&c,50,CURRENT_CAL_REFERENCE_DEFAULT,1,&in));assert(c.drive_angle==0);
 puts("PASS: precheck unpowered, hinted entry continuous, shortest slew, final phase zero, fallback and range");
}
