#include "current_limits.h"
#include "motor_calibration.h"
#include <stddef.h>
#include <string.h>
#include <limits.h>

static int32_t abs32(int32_t v) { return v<0 ? -v : v; }
static int32_t modulo(int32_t v) { v%=16384; return v<0?v+16384:v; }
static void enter(motor_cal_t *c, motor_cal_state_t state, uint32_t now)
{
  c->state=state; c->entered=now; c->samples=0;
  c->sample_sum=0; c->current_sum=0;
  c->sample_min=INT32_MAX; c->sample_max=INT32_MIN;
}
void motor_cal_init(motor_cal_t *c) { if(c) memset(c,0,sizeof(*c)); }
uint8_t motor_cal_active(const motor_cal_t *c)
{ return (uint8_t)(c && c->state>=MOTOR_CAL_PRECHECK && c->state<=MOTOR_CAL_REVERSE); }
void motor_cal_abort(motor_cal_t *c, motor_cal_error_t reason)
{
  if(!c || !motor_cal_active(c)) return;
  c->error=reason; c->drive=0; c->drive_reference=0;
  c->state=reason==MOTOR_CAL_USER?MOTOR_CAL_ABORTED:MOTOR_CAL_FAILED;
  c->candidate.magic=0;
}
/* 仅质量复核失败采用零电流退出。真正故障和用户中止仍立即关断。
 * 候选立即作废，电流清零后才上报质量失败，不允许保存。 */
static void reject_quality(motor_cal_t *c, motor_cal_error_t reason)
{
  if(c->drive && !c->releasing && c->drive_reference>0) {
    c->candidate.magic=0;c->pending_error=(uint8_t)reason;
    c->drive_target=c->drive_angle;c->releasing=1;
    c->release_started=c->last_sample;c->release_settled=0;return;
  }
  motor_cal_abort(c,reason);
}
uint8_t motor_cal_start(motor_cal_t *c, uint16_t poles, uint16_t current,
                        uint8_t full, const motor_cal_input_t *in)
{
  if(!c || !in || motor_cal_active(c) || (poles!=50 && poles!=100) ||
     current<CURRENT_CAL_REFERENCE_MIN || current>CURRENT_CAL_REFERENCE_MAX || full>2 || !in->sensors_ready ||
     in->bus_mv<10000 || in->bus_mv>30000 ||
     (in->alignment_hint_valid && in->alignment_angle_mdeg>=360000U)) return 0;
  motor_cal_init(c);
  c->candidate.pole_pairs=poles; c->current=current; c->full=full;
  c->started=c->speed_tick=in->now_ms;
  c->start_counts=c->speed_counts=in->counts;
  c->last_sample=in->now_ms-1;
  if(full!=2U && in->alignment_hint_valid) {
    /* The hint only initializes the field; phase zero is still re-measured.
     * Precheck stays unpowered, then the existing slew converges to zero. */
    c->drive_angle=(int32_t)in->alignment_angle_mdeg;
    c->drive_target=c->drive_angle;
  }
  enter(c,MOTOR_CAL_PRECHECK,in->now_ms);
  return 1;
}
static uint8_t sample(motor_cal_t *c, const motor_cal_input_t *in, uint32_t hold)
{
  uint32_t dt=in->now_ms-c->entered;
  if(dt+100U>=hold) {
    c->sample_sum+=in->counts; c->current_sum+=in->id_actual; c->samples++;
    if(in->counts<c->sample_min)c->sample_min=in->counts;
    if(in->counts>c->sample_max)c->sample_max=in->counts;
  }
  if(dt<hold)return 0;
  if(c->samples<30 || c->sample_max-c->sample_min>8) {
    motor_cal_abort(c,MOTOR_CAL_UNSTABLE); return 0;
  }
  if(c->drive && c->current_sum*100 < (int64_t)c->current*c->samples*55) {
    motor_cal_abort(c,MOTOR_CAL_CURRENT); return 0;
  }
  return 1;
}
static void finish(motor_cal_t *c)
{
  /* 正常完成先缓慢撤流，防止储能在关桥瞬间回灌母线。
   * 故障/用户中止仍立即关桥，不延迟安全路径。 */
  if(c->drive && !c->releasing) {
    c->releasing=1;c->release_started=c->last_sample;
    c->release_settled=0;return;
  }
  c->candidate.flags=c->full?3U:1U;
  motor_cal_seal(&c->candidate,0);
  if(!motor_cal_record_valid(&c->candidate)) {motor_cal_abort(c,MOTOR_CAL_MAP);return;}
  c->drive=0;c->drive_reference=0;c->progress=1000;c->state=MOTOR_CAL_DONE;
}
void motor_cal_step(motor_cal_t *c, const motor_cal_input_t *in)
{
  uint32_t dt;
  int32_t measured,delta;
  if(!motor_cal_active(c) || !in || in->now_ms==c->last_sample)return;
  if(in->now_ms-c->last_sample>20U) {motor_cal_abort(c,MOTOR_CAL_SENSOR);return;}
  /* Slew the forced field instead of stepping an entire LUT interval.
     At 50 pole pairs this caps commanded mechanical speed at 10 deg/s. */
  dt=in->now_ms-c->last_sample;
  delta=c->drive_target-c->drive_angle;
  if(delta>180000)delta-=360000;
  if(delta< -180000)delta+=360000;
  if(delta>(int32_t)dt*500)delta=(int32_t)dt*500;
  if(delta< -(int32_t)dt*500)delta=-(int32_t)dt*500;
  c->drive_angle=(c->drive_angle+delta+360000)%360000;
  c->last_sample=in->now_ms;
  if(!in->sensors_ready) {motor_cal_abort(c,MOTOR_CAL_SENSOR);return;}
  if(in->bus_mv<10000 || in->bus_mv>30000) {motor_cal_abort(c,MOTOR_CAL_POWER);return;}
  if(abs32(in->phase_a)>CURRENT_CAL_PHASE_TRIP || abs32(in->phase_b)>CURRENT_CAL_PHASE_TRIP) {motor_cal_abort(c,MOTOR_CAL_CURRENT);return;}
  if(in->now_ms-c->started>220000U) {motor_cal_abort(c,MOTOR_CAL_TIMEOUT);return;}
  delta=in->counts-c->start_counts;
  if(abs32(delta)>(c->full?17750:450)) {motor_cal_abort(c,MOTOR_CAL_TRAVEL);return;}
  dt=in->now_ms-c->speed_tick;
  if(dt>=50U) {
    if((int64_t)abs32(in->counts-c->speed_counts)*1000 > (int64_t)dt*1366) {
      motor_cal_abort(c,MOTOR_CAL_TRAVEL);return;
    }
    c->speed_tick=in->now_ms;c->speed_counts=in->counts;
  }
  if(c->releasing) {
    uint32_t elapsed=in->now_ms-c->release_started;
    if(elapsed>1500U) {motor_cal_abort(c,MOTOR_CAL_CURRENT);return;}
    c->drive_reference=elapsed>=400U?0:
      (int32_t)((uint32_t)c->current*(400U-elapsed)/400U);
    if(c->drive_reference==0 && abs32(in->id_actual)<=3 &&
       abs32(in->phase_a)<=4 && abs32(in->phase_b)<=4) {
      if(!c->release_settled)c->release_settled=in->now_ms;
      if(in->now_ms-c->release_settled>=30U) {
        if(c->pending_error)motor_cal_abort(c,(motor_cal_error_t)c->pending_error);
        else finish(c);
      }
    } else c->release_settled=0;
    return;
  }
  dt=in->now_ms-c->entered;
  if(c->state==MOTOR_CAL_PRECHECK) {
    if(abs32(in->phase_a)>8 || abs32(in->phase_b)>8) {motor_cal_abort(c,MOTOR_CAL_CURRENT);return;}
    if(sample(c,in,500)) {
      if(c->full==2){c->state=MOTOR_CAL_STATIC_OK;c->progress=1000;return;}
      c->drive=1;c->drive_target=0;enter(c,MOTOR_CAL_ALIGN,in->now_ms);
    }
  } else if(c->state==MOTOR_CAL_ALIGN) {
    c->drive_reference=(int32_t)c->current*(int32_t)(dt>1000?1000:dt)/1000;
    if(sample(c,in,1500)) {
      c->zero_counts=(int32_t)(c->sample_sum/c->samples);
      c->index=1;c->drive_target=90000;c->progress=20;enter(c,MOTOR_CAL_DIRECTION,in->now_ms);
    }
  } else if(c->state==MOTOR_CAL_DIRECTION) {
    if(sample(c,in,500)) {
      /* Compare the same electrical phase after a full electrical cycle.
         A single quarter-cycle confounds direction with detent/static friction.
         Each quarter is still slewed, current-checked and required to settle. */
      if(c->index<4U) {
        c->index++;c->drive_target=(int32_t)(c->index*90000U%360000U);
        c->progress=(uint16_t)(20U+c->index*2U);
        enter(c,MOTOR_CAL_DIRECTION,in->now_ms);return;
      }
      measured=(int32_t)(c->sample_sum/c->samples)-c->zero_counts;
      delta=16384/(int32_t)c->candidate.pole_pairs;
      if(abs32(abs32(measured)-delta)>delta/4) {motor_cal_abort(c,MOTOR_CAL_NO_MOTION);return;}
      c->candidate.direction=measured>0?1:-1;
      c->index=3;c->drive_target=270000;c->progress=30;
      enter(c,MOTOR_CAL_RETURN,in->now_ms);
    }
  } else if(c->state==MOTOR_CAL_RETURN) {
    if(sample(c,in,500)) {
      if(c->index>0U) {
        c->index--;c->drive_target=(int32_t)c->index*90000;
        c->progress=(uint16_t)(30U+(3U-c->index)*2U);
        enter(c,MOTOR_CAL_RETURN,in->now_ms);return;
      }
      measured=(int32_t)(c->sample_sum/c->samples);
      delta=abs32(measured-c->zero_counts);
      if(delta>8) {reject_quality(c,MOTOR_CAL_HYSTERESIS);return;}
      c->zero_counts=(measured+c->zero_counts)/2;
      c->candidate.raw_zero=(uint16_t)modulo(c->zero_counts);
      c->candidate.hysteresis_counts=(uint16_t)delta;
      for(c->index=0;c->index<=MOTOR_CAL_BINS;c->index++)c->candidate.knots[c->index]=(int16_t)(c->index*64);
      if(!c->full) {finish(c);return;}
      c->index=1;c->drive_target=(int32_t)((uint32_t)c->candidate.pole_pairs*360000U/MOTOR_CAL_BINS%360000U);
      enter(c,MOTOR_CAL_FORWARD,in->now_ms);
    }
  } else if(c->state==MOTOR_CAL_FORWARD || c->state==MOTOR_CAL_REVERSE) {
    if(!sample(c,in,400))return;
    measured=((int32_t)(c->sample_sum/c->samples)-c->zero_counts)*c->candidate.direction;
    if(abs32(measured-(int32_t)c->index*64)>128) {reject_quality(c,MOTOR_CAL_MAP);return;}
    if(c->state==MOTOR_CAL_FORWARD) {
      c->candidate.knots[c->index]=(int16_t)measured;
      c->progress=(uint16_t)(40+c->index*450/MOTOR_CAL_BINS);
      if(c->index==MOTOR_CAL_BINS)enter(c,MOTOR_CAL_REVERSE,in->now_ms);
      else {c->index++;enter(c,MOTOR_CAL_FORWARD,in->now_ms);}
    } else {
      delta=abs32(measured-c->candidate.knots[c->index]);
      if(delta>8) {reject_quality(c,MOTOR_CAL_HYSTERESIS);return;}
      if(delta>c->candidate.hysteresis_counts)c->candidate.hysteresis_counts=(uint16_t)delta;
      c->candidate.knots[c->index]=(int16_t)((measured+c->candidate.knots[c->index])/2);
      c->progress=(uint16_t)(490+(MOTOR_CAL_BINS-c->index)*500/MOTOR_CAL_BINS);
      if(c->index==0) {
        if(abs32(c->candidate.knots[0])>8 || abs32(c->candidate.knots[MOTOR_CAL_BINS]-16384)>8) {reject_quality(c,MOTOR_CAL_MAP);return;}
        c->candidate.knots[0]=0;c->candidate.knots[MOTOR_CAL_BINS]=16384;
        finish(c);return;
      }
      c->index--;enter(c,MOTOR_CAL_REVERSE,in->now_ms);
    }
    c->drive_target=(int32_t)((uint64_t)c->index*c->candidate.pole_pairs*360000U/MOTOR_CAL_BINS%360000U);
  }
}
uint32_t motor_cal_crc(const uint8_t *p,uint32_t n)
{
  uint32_t crc=0xffffffffU,i;uint8_t b;
  for(i=0;i<n;i++){crc^=p[i];for(b=0;b<8;b++)crc=(crc>>1)^((crc&1)?0xedb88320U:0);}
  return ~crc;
}
void motor_cal_seal(motor_cal_record_t *r,uint32_t seq)
{
  r->magic=MOTOR_CAL_MAGIC;r->version=MOTOR_CAL_VERSION;r->sequence=seq;r->reserved=0;
  r->crc=motor_cal_crc((const uint8_t *)r,(uint32_t)offsetof(motor_cal_record_t,crc));
}
uint8_t motor_cal_record_valid(const motor_cal_record_t *r)
{
  uint16_t j;
  if(!r || r->magic!=MOTOR_CAL_MAGIC || r->version!=MOTOR_CAL_VERSION ||
     (r->flags!=1 && r->flags!=3) || (r->pole_pairs!=50 && r->pole_pairs!=100) ||
     r->raw_zero>=16384 || (r->direction!=1 && r->direction!=-1) || r->hysteresis_counts>8 ||
     r->knots[0]!=0 || r->knots[MOTOR_CAL_BINS]!=16384 ||
     r->crc!=motor_cal_crc((const uint8_t *)r,(uint32_t)offsetof(motor_cal_record_t,crc)))return 0;
  for(j=1;j<=MOTOR_CAL_BINS;j++)if(r->knots[j]-r->knots[j-1]<16 || r->knots[j]-r->knots[j-1]>112)return 0;
  return 1;
}
static uint64_t corrected_counts(const motor_cal_record_t *r,uint16_t raw)
{
  uint16_t lo=0,hi=MOTOR_CAL_BINS,mid;
  int32_t relative=modulo(((int32_t)(raw&16383)-r->raw_zero)*r->direction);
  uint64_t corrected;
  /* Record validity is checked once on load, never CRC-scanned in the loop. */
  while(hi-lo>1) {mid=(uint16_t)((lo+hi)/2);if(r->knots[mid]<=relative)lo=mid;else hi=mid;}
  corrected=((uint64_t)lo*64U<<16) + ((uint64_t)(relative-r->knots[lo])*64U*65536U)/(uint32_t)(r->knots[hi]-r->knots[lo]);
  return corrected;
}

uint32_t motor_cal_angle(const motor_cal_record_t *r,uint16_t raw)
{ return (uint32_t)((corrected_counts(r,raw)*r->pole_pairs*360000U/(16384ULL*65536U))%360000U); }
uint16_t motor_cal_corrected_raw(const motor_cal_record_t *r,uint16_t raw)
{ return (uint16_t)modulo((int32_t)r->raw_zero+r->direction*(int32_t)((corrected_counts(r,raw)+32768U)/65536U)); }
