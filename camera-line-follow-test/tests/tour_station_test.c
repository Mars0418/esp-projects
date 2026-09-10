#include <assert.h>
#include <stdio.h>
#include "tour_corner.h"
#include "tour_speech.h"
int main(void) {
    assert(tour_station_clip(1,1,false)==1);
    assert(tour_station_clip(1,2,false)==2);
    assert(tour_station_clip(1,2,true)==3);
    assert(tour_station_clip(2,1,false)==4);
    assert(tour_station_clip(2,1,true)==5);
    assert(tour_station_clip(1,1,true)==-1);
    assert(tour_station_clip(2,2,false)==-1);
    assert(tour_station_clip(0,1,false)==-1);
    assert(!tour_station_departure_allowed(false,90000000,2000000,false));
    assert(!tour_station_departure_allowed(true,1999999,2000000,false));
    assert(!tour_station_departure_allowed(true,90000000,2000000,true));
    assert(tour_station_departure_allowed(true,2000000,2000000,false));
    puts("PASS: both routes' announcements, invalid stops, manual-only release, speech interlock");
}
