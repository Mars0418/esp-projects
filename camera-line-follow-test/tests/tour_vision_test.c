#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "line_vision.h"
static int route;
int tour_guide_route(void) { return route; }
static unsigned char frame[80*60*2];
static void point(int x, int y) {
    for (int dx=-1; dx<=1; ++dx) {
        int p = (y*80+x+dx)*2;
        frame[p] = frame[p+1] = 0;
    }
}
static line_vision_result_t run(int choice, int shape) {
    route=choice;
    memset(frame,255,sizeof(frame));
    for(int y=0;y<51;++y) {
        if(shape==0 || shape==3) {
            int d=y<6?0:(y-6)*25/44;
            point(40-d,y);
            if(shape==0 || y<32) point(40+d,y);
        } else if(shape==1) point(40,y);
    }
    line_vision_result_t r;
    line_vision_process(frame,80,60,&r);
    return r;
}
int main(void) {
    assert(line_vision_init(80,60)==ESP_OK);
    line_vision_result_t left=run(1,0),right=run(2,0);
    printf("Y: left=%d right=%d valid=%d/%d\n",left.lookahead_x,right.lookahead_x,left.foot_track_valid,right.foot_track_valid);
    assert(left.foot_track_valid && right.foot_track_valid);
    assert(left.lookahead_x>45 && right.lookahead_x<35);
    left=run(1,3);right=run(2,3);
    assert(left.lookahead_x>45 && right.lookahead_x<35);
    for(int i=1;i<=2;++i) {
        line_vision_result_t straight=run(i,1);
        assert(straight.foot_track_valid && straight.lookahead_x>=38 && straight.lookahead_x<=42);
        line_vision_result_t end=run(i,2);
        assert(!end.found && !end.foot_track_valid);
    }
    line_vision_deinit();
    puts("PASS: both Y arms, straight tracking, blank endpoints");
}
