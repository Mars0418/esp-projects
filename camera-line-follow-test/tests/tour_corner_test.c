#include <assert.h>
#include <stdio.h>
#include "tour_corner.h"
int main(void) {
    for (int corner = 0; corner < 2; ++corner) {
        int count = 0;
        int64_t start = corner * 10000000LL;
        int64_t ready = start + 2500000;
        for (int i=0; i<25; ++i)
            assert(!tour_corner_rearm_ready(start+i*100000,ready,true,true,&count));
        assert(!tour_corner_rearm_ready(ready,ready,true,true,&count));
        for(int i=0;i<10;++i)
            assert(!tour_corner_rearm_ready(ready,ready,false,true,&count));
        assert(!tour_corner_rearm_ready(ready+100000,ready,true,false,&count));
        assert(count==0);
        assert(!tour_corner_rearm_ready(ready+200000,ready,true,true,&count));
        assert(!tour_corner_rearm_ready(ready+300000,ready,true,true,&count));
        assert(tour_corner_rearm_ready(ready+400000,ready,true,true,&count));
    }
    puts("PASS: two successive corners; cooldown, distinct frames, corner/no-line rejection");
}
