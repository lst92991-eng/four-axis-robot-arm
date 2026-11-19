#include "motor_calibration.h"
#include "current_limits.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static motor_cal_t cal;
static motor_cal_input_t in;
static int64_t field;
static int32_t previous_angle;
static uint16_t poles;
static int direction;
static int nonlinear;
static int detent_counts;
static int return_bias;
static void begin(uint16_t p,int dir,uint8_t full)
{
  motor_cal_init(&cal);memset(&in,0,sizeof(in));
  in.now_ms=100;in.counts=12000;in.bus_mv=24000;in.sensors_ready=1;
  field=0;previous_angle=0;poles=p;direction=dir;nonlinear=0;detent_counts=0;return_bias=0;
  assert(motor_cal_start(&cal,p,30,full,&in));
}
static void tick(void)
{
  int32_t delta=cal.drive_angle-previous_angle;
  if(delta>180000)delta-=360000;
  if(delta< -180000)delta+=360000;
  assert(delta>=-500 && delta<=500);
  field+=delta;previous_angle=cal.drive_angle;
  int32_t target=12000+direction*(int32_t)llround((double)field*16384.0/(poles*360000.0));
  target-=direction*(int32_t)llround(detent_counts*sin((double)field*6.283185307179586/360000.0));
  if(cal.state==MOTOR_CAL_RETURN)target+=return_bias;
  if(nonlinear)target+=(int32_t)llround(12.0*sin((target-12000)*6.283185307179586/16384.0));
  in.counts+=(target>in.counts)?1:((target<in.counts)?-1:0);
  in.id_actual=cal.drive_reference;
  in.now_ms++;
  motor_cal_step(&cal,&in);
}
static void complete(void)
{
  while(motor_cal_active(&cal) && in.now_ms<230000)tick();
  if(cal.state!=MOTOR_CAL_DONE)printf("state=%d error=%d progress=%u\n",cal.state,cal.error,cal.progress);
  assert(cal.state==MOTOR_CAL_DONE && !cal.drive);
  assert(motor_cal_record_valid(&cal.candidate));
}
static void basic_and_full(void)
{
  for(uint16_t p=50;p<=100;p+=50)for(int d=-1;d<=1;d+=2)for(int f=0;f<2;f++) {
    begin(p,d,(uint8_t)f);complete();
    assert(cal.candidate.direction==d && cal.candidate.pole_pairs==p);
    assert(cal.candidate.flags==(f?3U:1U));
    assert(cal.candidate.raw_zero==12000);
    assert(motor_cal_angle(&cal.candidate,12000)==0);
    for(unsigned raw=0;raw<16384;raw++)assert(motor_cal_corrected_raw(&cal.candidate,(uint16_t)raw)==raw);
  }
}
static void faults(void)
{
  begin(50,1,0);in.now_ms++;in.phase_a=CURRENT_CAL_PHASE_TRIP+1;motor_cal_step(&cal,&in);
  assert(cal.error==MOTOR_CAL_CURRENT && !cal.drive);
  begin(50,1,0);in.now_ms++;in.sensors_ready=0;motor_cal_step(&cal,&in);assert(cal.error==MOTOR_CAL_SENSOR);
  begin(50,1,0);in.now_ms+=25;motor_cal_step(&cal,&in);assert(cal.error==MOTOR_CAL_SENSOR);
  begin(50,1,0);in.now_ms++;in.bus_mv=35000;motor_cal_step(&cal,&in);assert(cal.error==MOTOR_CAL_POWER);
  begin(50,1,0);motor_cal_abort(&cal,MOTOR_CAL_USER);assert(cal.state==MOTOR_CAL_ABORTED && !cal.drive);
  begin(50,1,0);motor_cal_abort(&cal,MOTOR_CAL_LINK);assert(cal.state==MOTOR_CAL_FAILED);
  begin(50,1,0);
  while(motor_cal_active(&cal)) {
    in.now_ms++;in.id_actual=cal.drive_reference;motor_cal_step(&cal,&in);
  }
  assert(cal.error==MOTOR_CAL_NO_MOTION);
  begin(50,1,0);in.now_ms++;in.counts+=500;motor_cal_step(&cal,&in);assert(cal.error==MOTOR_CAL_TRAVEL);
  begin(50,1,0);while(cal.state!=MOTOR_CAL_ALIGN)tick();
  while(motor_cal_active(&cal)){in.now_ms++;in.id_actual=0;motor_cal_step(&cal,&in);}
  assert(cal.error==MOTOR_CAL_CURRENT);
}
static void nonlinear_mapping(void)
{
  for(int dir=-1;dir<=1;dir+=2){
    begin(50,dir,1);nonlinear=1;complete();
    for(int ideal=0;ideal<16384;ideal+=7){
      int measured=12000+ideal+(int)llround(12.0*sin(ideal*6.283185307179586/16384.0));
      int corrected=motor_cal_corrected_raw(&cal.candidate,(uint16_t)(measured&16383));
      int error=(corrected-((12000+ideal)&16383));
      if(error>8192)error-=16384;
      if(error< -8192)error+=16384;
      assert(error>=-1 && error<=1);
    }
  }
  begin(50,1,0);poles=100;
  while(motor_cal_active(&cal))tick();
  assert(cal.error==MOTOR_CAL_NO_MOTION);
}
static void periodic_detent_direction(void)
{
  /* A 40-count periodic static offset makes the former 90-degree test fail
     (41 counts vs the 61-count minimum), but cancels at one full cycle.
     This is a sensitivity model, not a claim to model the physical motor. */
  for(int dir=-1;dir<=1;dir+=2) {
    begin(50,dir,0);detent_counts=40;complete();
    assert(cal.candidate.direction==dir && cal.candidate.raw_zero==12000);
  }
  begin(50,1,0);return_bias=20;
  while(motor_cal_active(&cal))tick();
  assert(cal.error==MOTOR_CAL_HYSTERESIS && !cal.drive);
  begin(50,1,0);poles=100;detent_counts=20;
  while(motor_cal_active(&cal))tick();
  assert(cal.error==MOTOR_CAL_NO_MOTION);
}
static void integrity(void)
{
  begin(50,1,1);complete();
  motor_cal_record_t good=cal.candidate;
  for(unsigned i=0;i<sizeof(good);i++) {
    cal.candidate=good;((uint8_t *)&cal.candidate)[i]^=1;
    assert(!motor_cal_record_valid(&cal.candidate));
  }
  cal.candidate=good;cal.candidate.knots[10]=cal.candidate.knots[9];
  motor_cal_seal(&cal.candidate,1);assert(!motor_cal_record_valid(&cal.candidate));
  assert(motor_cal_crc((const uint8_t *)"123456789",9)==0xcbf43926U);
}
static void pending_quality(void)
{
  begin(50,1,0);return_bias=20;
  while(motor_cal_active(&cal) && !cal.pending_error)tick();
  assert(cal.pending_error==MOTOR_CAL_HYSTERESIS && cal.releasing && cal.drive);
  assert(cal.error==MOTOR_CAL_OK && !motor_cal_record_valid(&cal.candidate));
}
static void quality_ramp_guards(void)
{
  pending_quality();
  int32_t angle=cal.drive_angle, prior=cal.drive_reference;
  while(motor_cal_active(&cal)) {
    tick();assert(cal.drive_angle==angle && cal.drive_reference<=prior);
    assert(!motor_cal_record_valid(&cal.candidate));prior=cal.drive_reference;
  }
  assert(cal.state==MOTOR_CAL_FAILED && cal.error==MOTOR_CAL_HYSTERESIS);
  assert(!cal.drive && !cal.drive_reference && in.now_ms-cal.release_started>=400);
  for(int fault=0;fault<8;fault++) {
    pending_quality();in.now_ms++;
    if(fault==0)in.phase_a=CURRENT_CAL_PHASE_TRIP+1;
    if(fault==1)in.sensors_ready=0;
    if(fault==2)in.bus_mv=35000;
    if(fault==3)in.now_ms+=25;
    if(fault==4)in.counts=cal.start_counts+501;
    if(fault==5)cal.started=in.now_ms-220001U;
    if(fault==6)motor_cal_abort(&cal,MOTOR_CAL_USER);
    if(fault==7)motor_cal_abort(&cal,MOTOR_CAL_LINK);
    motor_cal_step(&cal,&in);
    assert(!motor_cal_active(&cal) && !cal.drive && !cal.drive_reference);
    assert(cal.error!=MOTOR_CAL_OK && !motor_cal_record_valid(&cal.candidate));
  }
  /* 电流退出无法达到零时仍强制关断。 */
  pending_quality();
  while(motor_cal_active(&cal)) {in.now_ms++;in.id_actual=10;motor_cal_step(&cal,&in);}
  assert(cal.error==MOTOR_CAL_CURRENT && !cal.drive);
}
int main(void)
{
  begin(50,1,2);
  while(motor_cal_active(&cal)){tick();assert(!cal.drive && !cal.drive_reference);}
  assert(cal.state==MOTOR_CAL_STATIC_OK && !motor_cal_record_valid(&cal.candidate));
  quality_ramp_guards();
  basic_and_full();faults();nonlinear_mapping();periodic_detent_direction();integrity();
  puts("motor_calibration: PASS (8 profiles, faults, CRC, periodic mapping)");return 0;
}
