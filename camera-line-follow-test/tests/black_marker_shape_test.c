#include "black_marker_vision.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned char pixels[160*120*2];
static void ellipse(unsigned short color) {
    memset(pixels,255,sizeof(pixels));
    for(int y=0;y<8;y++) for(int x=0;x<40;x++) {
        if(x*x/1600.0f+y*y/64.0f>1) continue;
        int at=2*((50+y)*160+48+x);
        pixels[at]=color>>8; pixels[at+1]=color;
    }
}
int main(int argc, char **argv) {
    black_marker_result_t r;
    assert(black_marker_vision_init(160,120)==0);
    ellipse(0); black_marker_vision_process(pixels,160,120,&r);
    assert(r.found && !r.predicted && r.confidence>=60);
    assert(black_marker_vision_init(160,120)==0);
    memset(pixels,255,sizeof(pixels));
    for(int x=10;x<140;x++) for(int y=50;y<52;y++) {
        pixels[2*(y*160+x)]=pixels[2*(y*160+x)+1]=0;
    }
    black_marker_vision_process(pixels,160,120,&r); assert(!r.found);
    assert(black_marker_vision_init(160,120)==0);
    ellipse((5<<11)|8); black_marker_vision_process(pixels,160,120,&r); assert(!r.found);
    // Far and side goals outside the old central ROI remain valid when complete.
    assert(black_marker_vision_init(160,120)==0);
    ellipse(0);
    unsigned char shifted[sizeof(pixels)]; memset(shifted,255,sizeof(shifted));
    for(int y=0;y<8;y++) for(int x=0;x<40;x++) {
        int src=2*((50+y)*160+48+x), dst=2*((106+y)*160+105+x);
        shifted[dst]=pixels[src]; shifted[dst+1]=pixels[src+1];
    }
    black_marker_vision_process(shifted,160,120,&r);
    assert(r.found && !r.predicted && r.center_y>104 && r.confidence>=60);
    assert(black_marker_vision_init(160,120)==0);
    memset(shifted,255,sizeof(shifted));
    for(int y=0;y<8;y++) for(int x=0;x<40;x++) {
        int src=2*((50+y)*160+48+x), dst=2*((50+y)*160+x);
        shifted[dst]=pixels[src]; shifted[dst+1]=pixels[src+1];
    }
    black_marker_vision_process(shifted,160,120,&r); assert(!r.found);
    // A four-pixel thin court stripe still fails the aspect test.
    assert(black_marker_vision_init(160,120)==0);
    memset(pixels,255,sizeof(pixels));
    for(int x=10;x<140;x++) for(int y=50;y<54;y++) {
        pixels[2*(y*160+x)]=pixels[2*(y*160+x)+1]=0;
    }
    black_marker_vision_process(pixels,160,120,&r); assert(!r.found);
    for (int argument=1; argument<argc; ++argument) {
        FILE *file=fopen(argv[argument],"rb"); assert(file);
        assert(fread(pixels,1,sizeof(pixels),file)==sizeof(pixels)); fclose(file);
        assert(black_marker_vision_init(160,120)==0);
        black_marker_vision_process(pixels,160,120,&r);
        if (strstr(argv[argument], "no_goal_floor")) { assert(!r.found); continue; }
        assert(r.found && !r.predicted && r.confidence>=55);
        assert(r.center_x>=60 && r.center_x<=80 && r.center_y>=103 && r.center_y<=110);
    }
    puts("PASS: perspective-flattened goal accepted; thin line and dark purple rejected");
}
