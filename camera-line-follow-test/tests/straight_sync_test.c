#include "straight_sync.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    straight_sync_t s={0};
    straight_sync_update(&s,true,100,200); assert(s.pwm_a==0 && s.pwm_d==0);
    straight_sync_update(&s,true,0,300); assert(s.pwm_a==0 && s.pwm_d==0);
    straight_sync_update(&s,true,-80,300); assert(s.pwm_a==-10 && s.pwm_d==10);
    straight_sync_update(&s,true,-80,540); assert(s.pwm_a==20 && s.pwm_d==-20);
    straight_sync_update(&s,true,-80,5000); assert(s.pwm_a==25 && s.pwm_d==-25);
    straight_sync_update(&s,false,-80,5000); assert(!s.active && s.pwm_a==0 && s.pwm_d==0);
    straight_sync_update(&s,true,-80,5000); assert(s.pwm_a==0);
    assert(straight_sync_distance(INT32_MIN+9,INT32_MAX-10)==20);
    assert(straight_sync_distance(INT32_MAX-10,INT32_MIN+9)==20);
    assert(straight_sync_push_duty(0,0)==250);
    assert(straight_sync_push_duty(10,-5)==255);
    assert(straight_sync_push_duty(-60,-25)==165);
    assert(straight_sync_push_duty(100,100)==335);
    assert(straight_sync_speed_target(180,180)==180);
    assert(straight_sync_speed_target(160,200)==180);
    assert(straight_sync_speed_target(0,0)==0);
    assert(straight_push_heading_pwm(0,0)==0);
    assert(straight_push_heading_pwm(0,0.003f)==0);
    assert(straight_push_heading_pwm(0,0.05235988f)==-9);
    assert(straight_push_heading_pwm(0,-0.05235988f)==9);
    assert(straight_push_heading_pwm(0,1)==-30);
    assert(straight_push_heading_pwm(0,-1)==30);
    assert(straight_push_heading_pwm(3.13f,-3.13f)<0);
    assert(straight_push_heading_pwm(-3.13f,3.13f)>0);
    assert(straight_push_heading_pwm(0,NAN)==0);
    puts("PASS: heading correction signs, deadband, wrap, limits; equal travel, both correction signs, saturation, reset and encoder wrap");
}
