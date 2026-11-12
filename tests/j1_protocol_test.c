#include "communication_CAN.h"
#include "calibration.h"
#include "motor_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

motion_control_t Joint;
static const JointConfig profile = {.can_node = 1, .default_speed_deg_s = 10};
const JointConfig *joint_config = &profile;
uint8_t motor_configured=1;
volatile uint32_t system_ms;
static CanPacket request,reply;
static int queued,fail_ack,moves,tunes,acks,resets;
static motor_cal_t cal;
int CAN_init(void){return 1;}
int CAN_receive(uint16_t *id,uint8_t data[64])
{if(!queued)return 0;queued=0;*id=0x401;memcpy(data,&request,64);return 1;}
int CAN_send(uint16_t id,const uint8_t data[64])
{if(id==0x481){if(fail_ack){fail_ack--;return 0;}memcpy(&reply,data,64);acks++;}return 1;}
int Motor_canRun(void){return 1;}
int Motor_clearFault(void){return 1;}
int Motion_setZero(int64_t counts,float degrees){(void)counts;(void)degrees;return 1;}
int Motion_moveTo(float target,float speed){(void)target;(void)speed;moves++;return 1;}
int Motion_runVelocity(float speed){(void)speed;moves++;return 1;}
int Motion_setBrakeAcceleration(float a){(void)a;return 1;}
float Motion_brakeAcceleration(void){return 360;}
int Motion_stop(int lost){(void)lost;return 1;}
void Motion_release(void){}
void Motion_clearFault(void){}
int Calibration_active(void){return 0;}
void Calibration_abort(motor_cal_error_t e){(void)e;}
int Calibration_command(int op,int ma,const motor_status_t *b,uint32_t now)
{(void)op;(void)ma;(void)b;(void)now;return 1;}
const motor_cal_t *Calibration_status(void){return &cal;}
const motor_cal_record_t *Calibration_record(void){return 0;}
int Calibration_ready(void){return 0;}
int Calibration_stored(void){return 0;}
int Motor_setSpeedGains(int32_t p,int32_t i){(void)p;(void)i;tunes++;return 1;}
void Motor_readSpeedGains(int32_t v[3]){v[0]=2000;v[1]=100000;v[2]=1000;}
void NVIC_SystemReset(void){resets++;}
static void command(int cmd,int seq,int value,motor_status_t *b,uint32_t now)
{
  memset(&request,0,sizeof(request));request.kind=(uint8_t)cmd;request.node=1;
  request.version=3;request.sequence=(uint32_t)seq;request.values[0]=value;
  queued=1;system_ms=now;CAN_protocol_process(b,now);
}
int main(void)
{
  motor_status_t b={0};b.encoder_valid=1;CAN_protocol_setup();
  fail_ack=1;command(CMD_MOVE,1,1000,&b,10);
  assert(moves==1 && acks==0);
  CAN_protocol_process(&b,11);assert(moves==1 && acks==1 && reply.result==0);
  command(CMD_MOVE,1,1000,&b,12);assert(moves==1 && reply.result==1);
  b.power=1;command(CMD_TUNE_SPEED,2,2000,&b,20);assert(tunes==0 && reply.result==1);
  b.power=0;command(CMD_TUNE_SPEED,3,2000,&b,21);
  assert(tunes==1 && reply.result==0 && reply.values[0]==2000 && reply.values[1]==100000);
  b.power=1;command(CMD_REBOOT,4,0x52454254,&b,30);assert(reply.result==1);
  b.power=0;command(CMD_REBOOT,5,0,&b,31);assert(reply.result==1);
  command(CMD_REBOOT,6,0x52454254,&b,40);assert(reply.result==0);
  CAN_protocol_process(&b,139);assert(resets==0);
  CAN_protocol_process(&b,140);assert(resets==1);
  puts("j1_protocol: PASS (ACK queue, no duplicate motion, stopped tuning/reboot gates)");
  return 0;
}
