#include "tour_start_turn.h"
#include "post_line_odometry_config.h"
#include <math.h>
#include <string.h>
void tour_start_turn_begin(tour_start_turn_t *s, int route, const int32_t counts[3], int64_t now)
{
    *s=(tour_start_turn_t){.target_deg=route==1?30.0f:-30.0f,.started_us=now,.stable_since_us=now};
    memcpy(s->start,counts,sizeof(s->start));
    memcpy(s->previous,counts,sizeof(s->previous));
    if(route!=1 && route!=2) s->failed=true;
}
int tour_start_turn_step(tour_start_turn_t *s, const int32_t counts[3], int64_t now)
{
    if(s->done || s->failed) return 0;
    const float k[3]={
        POST_ODOM_A_ENCODER_SIGN*360*POST_ODOM_A_WHEEL_RADIUS_MM/(POST_ODOM_A_COUNTS_PER_REV*POST_ODOM_A_POSITION_RADIUS_MM),
        POST_ODOM_B_ENCODER_SIGN*360*POST_ODOM_B_WHEEL_RADIUS_MM/(POST_ODOM_B_COUNTS_PER_REV*POST_ODOM_B_POSITION_RADIUS_MM),
        POST_ODOM_D_ENCODER_SIGN*360*POST_ODOM_D_WHEEL_RADIUS_MM/(POST_ODOM_D_COUNTS_PER_REV*POST_ODOM_D_POSITION_RADIUS_MM)};
    s->angle_deg=0;
    for(int i=0;i<3;++i) {
        s->angle_deg+=(int32_t)((uint32_t)counts[i]-(uint32_t)s->start[i])*k[i]/3;
        if(counts[i]!=s->previous[i]) s->stable_since_us=now;
        s->previous[i]=counts[i];
    }
    float error=s->target_deg-s->angle_deg;
    if(fabsf(error)<=2) {
        if(now-s->stable_since_us>=300000) s->done=true;
        if(s->done) return 0;
    }
    if(now-s->started_us>=10000000) {s->failed=true;return 0;}
    if(fabsf(error)<=2) return 0;
    /* Short drive pulses followed by braking reduce small-angle overshoot. */
    int64_t pulse=(now-s->started_us)%200000;
    if(pulse >= (fabsf(error)>10?60000:40000)) return 0;
    return error>0?1:-1;
}
