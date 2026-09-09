#include "simple_ball_push.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static simple_push_input_t input;
static simple_push_motion_t tick(simple_push_t *s, int64_t us){input.now_us+=us;return simple_push_update(s,&input);}
static void still(simple_push_motion_t m){assert(m.forward==0 && m.right==0 && m.turn==0);}
int main(void){
    input=(simple_push_input_t){.fresh=true};
    simple_push_t search={.state=SIMPLE_SEARCH};
    simple_push_motion_t sweep=tick(&search,200000);
    assert(sweep.turn<0 && sweep.forward==0 && sweep.duration_ms==70);
    still(tick(&search,200000));
    sweep=tick(&search,500000); assert(sweep.turn<0);
    search=(simple_push_t){.state=SIMPLE_SEARCH,.ball_index=1,.entered_us=input.now_us};
    sweep=tick(&search,200000); assert(sweep.turn>0 && sweep.duration_ms==70);

    simple_push_t s={.state=SIMPLE_PUSH};
    input=(simple_push_input_t){.fresh=true,.red=true,.goal=true,.red_x=80,.red_y=50,
        .ball_ground_valid=true,.goal_ground_valid=true,.ball_forward_mm=200,.goal_forward_mm=500,
        .goal_gap_mm=181};
    assert(tick(&s,200000).forward>0);
    input.goal_gap_mm=180; still(tick(&s,200000));
    assert(s.state==SIMPLE_BRAKE_NEXT && s.ball_index==1);
    assert(strcmp(s.reason,"RED_GOAL_STOP_SWITCH_PURPLE")==0);
    /* Even visible purple cannot bypass brake/settle and mandatory left turn. */
    still(tick(&s,200000));still(tick(&s,200000));
    simple_push_motion_t m=tick(&s,100000);
    still(m); assert(s.state==SIMPLE_RETRACE);
    input.red=false; // Reverse must not require either ball to remain visible.
    still(tick(&s,1000000)); assert(s.state==SIMPLE_RETRACE);
    input.retrace_complete=true;
    still(tick(&s,200000)); assert(s.state==SIMPLE_BRAKE_TURN);
    input.retrace_complete=false; input.red=true;
    still(tick(&s,400000));
    m=tick(&s,100000);
    assert(s.state==SIMPLE_SEARCH && m.turn>0 && m.forward==0);
    assert(strcmp(simple_push_status_name(&s),"SEARCH_PURPLE")==0);
    still(tick(&s,200000));still(tick(&s,200000));still(tick(&s,200000));
    input.red=false; // Only red remains in view: selected purple is missing.
    m=tick(&s,100000);assert(m.turn>0 && s.state==SIMPLE_SEARCH);
    input.red=true;input.goal_gap_mm=INFINITY;input.now_us+=700000;
    for(int i=0;i<3;i++)still(tick(&s,200000));
    assert(s.state==SIMPLE_APPROACH);
    for(int i=0;i<3;i++)still(tick(&s,200000));
    assert(tick(&s,200000).forward==1 && s.state==SIMPLE_PUSH);
    assert(strcmp(simple_push_status_name(&s),"PUSH_PURPLE")==0);
    input.goal_gap_mm=180;still(tick(&s,200000));assert(s.state==SIMPLE_DONE && s.ball_index==1);
    input.start=true;still(tick(&s,200000));assert(s.state==SIMPLE_DONE);
    s=(simple_push_t){.state=SIMPLE_BRAKE_NEXT,.ball_index=1,.entered_us=input.now_us};
    input.emergency=true;still(tick(&s,200000));assert(s.state==SIMPLE_STOPPED);
    input.emergency=false;input.fresh=false;
    s=(simple_push_t){.state=SIMPLE_BRAKE_NEXT,.ball_index=1,.entered_us=input.now_us};
    still(tick(&s,200000));assert(s.state==SIMPLE_STOPPED);
    input.fresh=true;input.red=false;
    s=(simple_push_t){.state=SIMPLE_PUSH,.entered_us=input.now_us};
    for(int i=0;i<3;i++)still(tick(&s,200000));
    assert(s.state==SIMPLE_STOPPED && s.ball_index==0); // Failure never starts round two.
    input.red=false; input.retrace_complete=false;
    s=(simple_push_t){.state=SIMPLE_RETRACE,.ball_index=1,.entered_us=input.now_us};
    still(tick(&s,15000000)); assert(s.state==SIMPLE_STOPPED);
    s=(simple_push_t){.state=SIMPLE_RETRACE,.ball_index=1,.entered_us=input.now_us};
    input.emergency=true; still(tick(&s,200000)); assert(s.state==SIMPLE_STOPPED);
    input.emergency=false; input.fresh=false;
    s=(simple_push_t){.state=SIMPLE_RETRACE,.ball_index=1,.entered_us=input.now_us};
    still(tick(&s,200000)); assert(s.state==SIMPLE_STOPPED);
    puts("PASS: retrace completion, missing ball during reverse, reverse timeout/emergency/stale frame; red stop at 180mm, brake/settle, left turn, purple-only search, shared alignment/push, final stop and interruption");
}
