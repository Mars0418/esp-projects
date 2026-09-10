#include "tour_serial_frame.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    tour_serial_frame_t s={0};
    const char *text="@WIFI,phone,passwordXF P@";
    for(size_t i=0;i<strlen(text);++i) assert(tour_serial_feed(&s,text[i])==1);
    assert(tour_serial_feed(&s,'\n')==2);
    assert(!strcmp(s.data,text+1));
    assert(tour_serial_feed(&s,'F')==0);
    tour_serial_feed(&s,'@'); tour_serial_feed(&s,'X');
    assert(tour_serial_feed(&s,3)==0 && !s.active);
    assert(tour_serial_feed(&s,'P')==0);
    tour_serial_feed(&s,'@');
    for(int i=0;i<300;i++) assert(tour_serial_feed(&s,'X')==1);
    assert(tour_serial_feed(&s,'\n')==1 && !s.active);
    assert(tour_serial_feed(&s,'@')==1);
    assert(tour_serial_feed(&s,'\n')==2);
    puts("PASS: command fragmentation, embedded motor keys, overflow, emergency escape");
}
