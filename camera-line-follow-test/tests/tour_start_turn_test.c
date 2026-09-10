#include <assert.h>
#include <stdio.h>
#include "tour_start_turn.h"
int main(void) {
    for(int route=1;route<=2;++route) {
        int sign=route==1?-1:1;
        int32_t counts[3]={0};
        tour_start_turn_t s;
        tour_start_turn_begin(&s,route,counts,1000000);
        assert(tour_start_turn_step(&s,counts,1000000)==-sign);
        assert(tour_start_turn_step(&s,counts,1100000)==0);
        for(int i=0;i<3;++i) counts[i]=sign*111;
        assert(tour_start_turn_step(&s,counts,2000000)==0 && !s.done);
        assert(tour_start_turn_step(&s,counts,2299999)==0 && !s.done);
        assert(tour_start_turn_step(&s,counts,2300000)==0 && s.done && !s.failed);
        tour_start_turn_begin(&s,route,(int32_t[3]){0},0);
        for(int i=0;i<3;++i) counts[i]=sign*130;
        assert(tour_start_turn_step(&s,counts,200000)==sign); /* overshoot correction */
        tour_start_turn_begin(&s,route,(int32_t[3]){0},0);
        assert(tour_start_turn_step(&s,(int32_t[3]){0},10000000)==0 && s.failed);
    }
    puts("PASS: left/right 30 degrees, pulse braking, stationary confirmation, overshoot, stall timeout");
}
