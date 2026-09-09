#include "simple_ball_push.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static simple_push_input_t in;
static simple_push_motion_t step(simple_push_t *s) {
    in.now_us += 200000;
    return simple_push_update(s, &in);
}
static void stopped(simple_push_motion_t m) {
    assert(m.forward == 0 && m.right == 0 && m.turn == 0);
}
static void reset_input(void) {
    in = (simple_push_input_t){.fresh=true, .red=true, .red_x=80, .red_y=50,
        .goal=true, .ball_ground_valid=true, .goal_ground_valid=true,
        .ball_forward_mm=300, .goal_forward_mm=700, .goal_distance_mm=700, .goal_gap_mm=INFINITY};
}
/* Closed-loop geometry exercises both lateral signs and rotation. A bounded approach is allowed before final alignment; on entry to PUSH both targets must be ahead. */
static void simulate(float bx, float by, float gx, float gy) {
    reset_input();
    simple_push_t s = {.state=SIMPLE_APPROACH};
    for (int i=0; i<145; ++i) {
        in.ball_right_mm=bx; in.ball_forward_mm=by;
        in.red_y=(int)(by / 4);
        in.goal_right_mm=gx; in.goal_forward_mm=gy;
        simple_push_motion_t m=step(&s);
        if (s.state==SIMPLE_PUSH) {
            assert(fabsf(bx)<=10 && fabsf(gx)<=20);
            assert(fabsf(gx-bx*gy/by)<=20 && by>=130 && in.red_y>=39 && in.red_y<=59);
            assert(m.forward>0 && m.turn==0 && m.right==0);
            return;
        }
        assert(s.state==SIMPLE_APPROACH && m.forward<=.4f && m.speed_scale==.50f);
        float dt=fminf(m.duration_ms/1000.f, .2f);
        float speed_ratio=m.speed_scale/.25f;
        float dx=m.right*200*dt*speed_ratio, dy=m.forward*200*dt*speed_ratio;
        bx-=dx; gx-=dx; by-=dy; gy-=dy;
        float angle=m.turn*2*dt*speed_ratio, c=cosf(angle), sn=sinf(angle);
        float x=bx*c+by*sn; by=-bx*sn+by*c; bx=x;
        x=gx*c+gy*sn; gy=-gx*sn+gy*c; gx=x;
    }
    fprintf(stderr,"alignment failed bx=%f by=%f gx=%f gy=%f reason=%s\n", bx,by,gx,gy,s.reason);
    assert(0);
}
int main(void) {
    reset_input(); simple_push_t s={0};
    for(int i=0;i<10;i++) stopped(step(&s));
    assert(s.state==SIMPLE_WAIT);
    in.start=true; stopped(step(&s)); in.start=false;
    for(int i=0;i<3;i++) stopped(step(&s));
    assert(s.state==SIMPLE_APPROACH);
    in.goal=false;
    for(int i=0;i<4;i++) stopped(step(&s));
    assert(s.state==SIMPLE_APPROACH);
    in.goal=true; in.goal_right_mm=120;
    simple_push_motion_t m=step(&s);
    assert(m.forward==0 && m.right<0 && m.turn==0); // Move opposite off-axis goal.
    in.now_us+=400000; in.goal_right_mm=-120;
    m=step(&s); assert(m.forward==0 && m.right>0);
    in.now_us+=400000; in.goal_right_mm=0; in.ball_right_mm=100;
    stopped(step(&s)); m=step(&s); assert(m.turn<0 && m.forward==0);
    in.ball_right_mm=-100;
    for(int i=0;i<3;i++) stopped(step(&s)); // Prevent frame-by-frame reversal.
    stopped(step(&s)); m=step(&s); assert(m.turn>0);
    in.now_us+=700000; in.ball_right_mm=0; in.ball_forward_mm=120;
    m=step(&s); assert(m.forward<0 && m.right==0);
    in.ball_forward_mm=140; assert(step(&s).forward<0); // Clearance hysteresis.
    in.ball_forward_mm=180;
    stopped(step(&s)); stopped(step(&s)); stopped(step(&s));
    m=step(&s); assert(s.state==SIMPLE_PUSH && m.forward==1.0f && m.duration_ms==500 && m.speed_scale==1.00f);
    in.red_x=106; in.red_y=13; in.goal=false;
    m=step(&s); assert(s.state==SIMPLE_PUSH && m.forward>0 && m.turn==0 && m.right==0);
    in.red=false; stopped(step(&s)); stopped(step(&s)); stopped(step(&s));
    assert(s.state==SIMPLE_STOPPED);
    in.red=true; in.start=true; stopped(step(&s)); assert(s.state==SIMPLE_STOPPED);
    reset_input(); s=(simple_push_t){.state=SIMPLE_PUSH,.ball_index=1};
    in.red_x=130; stopped(step(&s)); assert(s.state==SIMPLE_STOPPED);
    assert(strcmp(s.reason,"SHOT_BALL_OFF_AXIS")==0);
    reset_input(); s=(simple_push_t){.state=SIMPLE_PUSH,.ball_index=1};
    in.red_y=40; in.goal_distance_mm=140; in.goal_gap_mm=100;
    stopped(step(&s)); stopped(step(&s)); stopped(step(&s)); assert(s.state==SIMPLE_DONE);
    in.emergency=true; stopped(step(&s)); assert(s.state==SIMPLE_DONE);
    reset_input(); s=(simple_push_t){.state=SIMPLE_APPROACH};
    in.emergency=true; stopped(step(&s)); assert(s.state==SIMPLE_STOPPED);
    reset_input(); s=(simple_push_t){.state=SIMPLE_APPROACH};
    in.fresh=false; stopped(step(&s)); assert(s.state==SIMPLE_STOPPED);
    reset_input(); s=(simple_push_t){.state=SIMPLE_APPROACH};
    in.ball_ground_valid=false; stopped(step(&s)); assert(s.state==SIMPLE_APPROACH);
    in.ball_ground_valid=true; in.goal_right_mm=NAN; stopped(step(&s));
    in.now_us=30000000; stopped(step(&s)); assert(s.state==SIMPLE_APPROACH);
    in.now_us=3600000000LL; stopped(step(&s)); assert(s.state==SIMPLE_APPROACH);
    in.emergency=true; stopped(step(&s)); assert(s.state==SIMPLE_STOPPED);
    reset_input(); s=(simple_push_t){.state=SIMPLE_PUSH,.ball_index=1};
    in.now_us=15000000; stopped(step(&s)); assert(s.state==SIMPLE_STOPPED);
    reset_input(); s=(simple_push_t){.state=SIMPLE_SEARCH};
    in.now_us=15000000; stopped(step(&s)); assert(s.state==SIMPLE_STOPPED);
    reset_input(); s=(simple_push_t){.state=SIMPLE_APPROACH};
    in.red_y=85;
    m=step(&s); assert(m.forward==.4f && m.speed_scale==.5f && s.state==SIMPLE_APPROACH);
    in.red_y=50;
    for(int i=0;i<3;i++) stopped(step(&s));
    m=step(&s); assert(s.state==SIMPLE_PUSH && m.forward==1);
    reset_input(); s=(simple_push_t){.state=SIMPLE_APPROACH};
    in.red_y=30; m=step(&s); assert(m.forward<0);
    reset_input(); s=(simple_push_t){.state=SIMPLE_PUSH,.ball_index=1};
    in.goal_gap_mm=181; assert(step(&s).forward>0);
    in.goal_gap_mm=180; stopped(step(&s)); assert(s.state==SIMPLE_DONE);
    assert(strcmp(s.reason,"PURPLE_GOAL_APPROACH_STOP")==0);
    reset_input(); s=(simple_push_t){.state=SIMPLE_PUSH,.ball_index=1};
    in.goal=false; in.goal_gap_mm=0; assert(step(&s).forward>0);
    in.goal=true; in.goal_gap_mm=NAN; assert(step(&s).forward>0);
    reset_input(); s=(simple_push_t){.state=SIMPLE_APPROACH};
    in.goal_ground_valid=false; stopped(step(&s));
    assert(strcmp(s.reason,"WAIT_GOAL_CORNER")==0);
    // Previously accepted off-axis geometry must not trigger straight motion.
    reset_input(); s=(simple_push_t){.state=SIMPLE_APPROACH};
    in.ball_right_mm=14; in.goal_right_mm=30;
    for(int i=0;i<8;i++) { m=step(&s); assert(s.state==SIMPLE_APPROACH && m.forward==0); }
    simulate(0,300,140,700); simulate(0,300,-140,700);
    simulate(70,300,100,700); simulate(-70,300,-100,700);
    simulate(0,150,100,650); simulate(0,300,0,700);
    puts("PASS: bounded approach to centre/lower image band; bilateral geometry convergence; clearance; missing goal; straight shot; loss/emergency/stale/timeout");
}
