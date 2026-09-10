#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "tour_guide.h"
#include "digit_model.h"
#include "digit_vision.h"
static int prediction;
int digit_model_selftest(digit_model_workspace_t *w,float *error) { (void)w;*error=0;return 10; }
void digit_model_predict(digit_model_workspace_t *w,const uint8_t *in,float *l,float *p) {
    (void)w;(void)in;
    for(int i=0;i<10;++i) {l[i]=0;p[i]=i==prediction?0.9f:0.01f;}
}
digit_region_t digit_vision_prepare(digit_vision_workspace_t *w,const uint8_t *f,unsigned r,bool m,uint8_t *i) {
    (void)w;(void)f;(void)r;(void)m;(void)i;
    return (digit_region_t){.valid=true};
}
int main(int argc,char **argv) {
    int choice=argc>1?atoi(argv[1]):1;
    assert(tour_guide_init()==ESP_OK);
    assert(tour_guide_waiting() && !tour_guide_departure_ready(10000000));
    prediction=choice;
    for(int i=1;i<=3;++i) tour_guide_process_digit(NULL,i*10000);
    assert(tour_guide_route()==0);
    assert(tour_guide_begin());
    assert(!tour_guide_begin());
    for(int i=4;i<=6;++i) tour_guide_process_digit(NULL,i*10000);
    assert(tour_guide_route()==0);
    tour_guide_enable_scan(100000);
    assert(tour_guide_scanning());
    prediction=7;
    for(int i=1;i<=3;++i) tour_guide_process_digit(NULL,i*100000);
    assert(tour_guide_waiting());
    prediction=choice;
    tour_guide_process_digit(NULL,400000);
    tour_guide_process_digit(NULL,500000);
    assert(tour_guide_waiting());
    tour_guide_process_digit(NULL,600000);
    assert(tour_guide_route()==choice);
    assert(!tour_guide_departure_ready(2599999));
    assert(tour_guide_departure_ready(2600000));
    prediction=3-choice;
    for(int i=7;i<=10;++i) tour_guide_process_digit(NULL,i*100000);
    assert(tour_guide_route()==choice);
    tour_guide_stop();
    assert(tour_guide_route()==-1 && !tour_guide_departure_ready(9000000));
    for(int i=11;i<=15;++i) tour_guide_process_digit(NULL,i*100000);
    assert(tour_guide_route()==-1);
    puts("PASS: reject 7, confirm three frames, departure delay, route lock, stop lock");
}

esp_err_t camera_display_show_digit(const uint8_t *p, const uint8_t *i,
    const digit_region_t *r, int d, float score, bool stable) {
    (void)p;(void)i;(void)r;(void)d;(void)score;(void)stable;return ESP_OK;
}

esp_err_t camera_display_show_rotated_rgb565(const uint8_t *p, size_t w, size_t h) {(void)p;(void)w;(void)h; return ESP_OK;}
