#include "digit_runtime.h"
#include "digit_model.h"
#include "digit_vision.h"
#include "digit_gate.h"
#include "camera_display.h"
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static digit_model_workspace_t *model;
static digit_vision_workspace_t *vision;
static digit_gate_t gate;
static uint8_t input[784], injected[784];
static atomic_bool inject_ready, dump_requested, disconnected;
/* Calibrated against this car's camera: TFT-oriented crop needs 90 degrees CW. */
static atomic_uint rotation=1;
static atomic_bool mirror;
static unsigned last_settings;
static const char *TAG="DIGIT";

static int hexval(char c)
{
    if(c>='0'&&c<='9') return c-'0';
    if(c>='a'&&c<='f') return c-'a'+10;
    if(c>='A'&&c<='F') return c-'A'+10;
    return -1;
}
static void uart_task(void *unused)
{
    (void)unused;
    char line[1600]; int length=0;
    bool overflow=false;
    int64_t last_byte=0;
    while(true) {
        uint8_t c;
        if(uart_read_bytes(UART_NUM_0,&c,1,pdMS_TO_TICKS(100))!=1) {
            if(length && esp_timer_get_time()-last_byte>3000000) {
                length=0; overflow=false;
            }
            continue;
        }
        last_byte=esp_timer_get_time();
        if(c=='\r') continue;
        if(c!='\n') {
            if(length<(int)sizeof(line)-1) line[length++]=c;
            else overflow=true;
            continue;
        }
        line[length]='\0';
        if(!overflow && length==1) {
            switch(line[0]) {
            case 'R': case 'r':
                atomic_fetch_add(&rotation,1);
                ESP_LOGI(TAG,"INPUT_ROTATION=%u degrees",(atomic_load(&rotation)&3)*90); break;
            case 'M': case 'm':
                atomic_store(&mirror,!atomic_load(&mirror));
                ESP_LOGI(TAG,"INPUT_MIRROR=%d",atomic_load(&mirror)); break;
            case 'C': case 'c': atomic_store(&dump_requested,true); break;
            default: ESP_LOGI(TAG,"Commands (Enter): C=capture R=rotate-input M=mirror-input; motors disabled");
            }
        } else if(!overflow && length==1570 && line[0]=='I' && line[1]==' ') {
            if(!atomic_load(&inject_ready)) {
                bool valid=true;
                for(int i=0;i<784;++i) {
                    int hi=hexval(line[2+i*2]), lo=hexval(line[3+i*2]);
                    if(hi<0||lo<0) {valid=false;break;}
                    injected[i]=(hi<<4)|lo;
                }
                if(valid) atomic_store(&inject_ready,true);
                else ESP_LOGW(TAG,"DIGIT_TEST_ERROR invalid_hex");
            } else ESP_LOGW(TAG,"DIGIT_TEST_ERROR busy");
        }
        length=0; overflow=false;
    }
}

esp_err_t digit_runtime_init(void)
{
    /* Known TB6612 pins from the existing car project. STBY low first. */
    gpio_set_level(GPIO_NUM_5,0);
    gpio_set_direction(GPIO_NUM_5,GPIO_MODE_OUTPUT);
    const gpio_num_t motor_pins[]={6,15,7,11,9,10,40,42,41};
    for(unsigned i=0;i<sizeof(motor_pins)/sizeof(motor_pins[0]);++i) {
        gpio_set_level(motor_pins[i],0);
        gpio_set_direction(motor_pins[i],GPIO_MODE_OUTPUT);
    }
    model=heap_caps_calloc(1,sizeof(*model),MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    vision=heap_caps_calloc(1,sizeof(*vision),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if(!model||!vision) return ESP_ERR_NO_MEM;
    float error;
    int64_t start=esp_timer_get_time();
    int passed=digit_model_selftest(model,&error);
    const double average_ms=(esp_timer_get_time()-start)/10000.0;
    if(passed!=10||error>0.002f) return ESP_FAIL;
    digit_gate_reset(&gate);
    const uart_config_t uart={.baud_rate=115200,.data_bits=UART_DATA_8_BITS,
        .parity=UART_PARITY_DISABLE,.stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0,&uart));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0,43,44,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,4096,0,0,NULL,0));
    /* Print after switching UART drivers so queued boot text is not discarded. */
    ESP_LOGI(TAG,"MODEL_SELFTEST passed=%d/10 max_logit_error=%.7f average_inference_ms=%.2f scratch=%u",
             passed,error,average_ms,(unsigned)sizeof(*model));
    if(xTaskCreate(uart_task,"digit_uart",4096,NULL,2,NULL)!=pdPASS) return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG,"DIGIT_READY model=onnx-mnist-8-fp32 threshold=%d frames=%d motors=DISABLED commands=C/R/M+Enter",
             (int)(DIGIT_MIN_SCORE*100+0.5f),DIGIT_CONFIRM_FRAMES);
    return ESP_OK;
}

void digit_runtime_disconnected(void) { atomic_store(&disconnected,true); }

void digit_runtime_idle(void)
{
    if(atomic_exchange(&disconnected,false)) digit_gate_reset(&gate);
    if(!atomic_load(&inject_ready)) return;
    float logits[10], probabilities[10];
    int64_t start=esp_timer_get_time();
    digit_model_predict(model,injected,logits,probabilities);
    int best=0;
    for(int i=1;i<10;++i) if(probabilities[i]>probabilities[best]) best=i;
    char line[512];
    int len=snprintf(line,sizeof(line),"DIGIT_TEST digit=%d ms=%.2f logits=",best,
                      (esp_timer_get_time()-start)/1000.0);
    for(int i=0;i<10;++i) len+=snprintf(line+len,sizeof(line)-len,"%s%.7f",i?",":"",logits[i]);
    line[len++]='\n';
    /* Release only after reading all input bytes; next request waits for this response. */
    atomic_store(&inject_ready,false);
    uart_write_bytes(UART_NUM_0,line,len);
}

static void dump_frame(const uint8_t *frame)
{
    static const char hex[]="0123456789abcdef";
    char line[1600];
    const char *begin="DIGIT_FRAME_BEGIN 160 120\n";
    uart_write_bytes(UART_NUM_0,begin,strlen(begin));
    for(int y=0;y<120;++y) {
        int n=snprintf(line,sizeof(line),"ROW %d ",y);
        for(int x=0;x<320;++x) {
            unsigned v=frame[y*320+x]; line[n++]=hex[v>>4]; line[n++]=hex[v&15];
        }
        line[n++]='\n'; uart_write_bytes(UART_NUM_0,line,n);
    }
    int n=snprintf(line,sizeof(line),"INPUT ");
    for(int i=0;i<784;++i) { line[n++]=hex[input[i]>>4];line[n++]=hex[input[i]&15]; }
    line[n++]='\n';uart_write_bytes(UART_NUM_0,line,n);
    const char *end="DIGIT_FRAME_END\n";
    uart_write_bytes(UART_NUM_0,end,strlen(end));
}

void digit_runtime_process(const uint8_t *frame, int64_t captured_at_us)
{
    digit_runtime_idle();
    unsigned rot=atomic_load(&rotation)&3;
    bool mir=atomic_load(&mirror);
    unsigned settings=rot+4*mir;
    if(settings!=last_settings) { digit_gate_reset(&gate);last_settings=settings; }
    int64_t start=esp_timer_get_time();
    digit_region_t region=digit_vision_prepare(vision,frame,rot,mir,input);
    int64_t prep_done=esp_timer_get_time();
    float logits[10]={0}, probabilities[10]={0};
    int best=-1; float score=0,margin=0;
    if(region.valid) {
        digit_model_predict(model,input,logits,probabilities);
        best=0;int second=1;
        for(int i=1;i<10;++i) if(probabilities[i]>probabilities[best]) best=i;
        second=(best==0)?1:0;
        for(int i=0;i<10;++i) if(i!=best&&probabilities[i]>probabilities[second]) second=i;
        score=probabilities[best];margin=score-probabilities[second];
    }
    int64_t done=esp_timer_get_time();
    bool fresh=done-captured_at_us<=600000;
    int event=-1;
    if(fresh) event=digit_gate_update(&gate,best,region.valid,score,margin,done/1000);
    else { gate.streak=0;gate.candidate=-1; }
    bool accepted=fresh&&region.valid&&score>=DIGIT_MIN_SCORE;
    bool stable=accepted&&gate.streak>=DIGIT_CONFIRM_FRAMES;
    if(event>=0) {
        /* Future action controller can consume this single event. No PWM here. */
        ESP_LOGI(TAG,"DIGIT_EVENT digit=%d score=%.4f suggestion=%s motors=DISABLED",
                 event,score,event==1?"SPIN":event==2?"FORWARD":"NONE");
    }
    static int64_t last_report;
    if(done-last_report>=300000) {
        ESP_LOGI(TAG,"RESULT digit=%d score=%.3f margin=%.3f accepted=%d stable=%d latch=%d reason=%s bbox=%d,%d,%d,%d threshold=%d rotation=%u mirror=%d prep=%.1fms infer=%.1fms age=%lldms",
                 best,score,margin,accepted,stable,gate.latched,
                 !fresh?"STALE":!region.valid?region.reason:!accepted?"UNCERTAIN":"OK",
                 region.x,region.y,region.width,region.height,region.threshold,rot*90,mir,
                 (prep_done-start)/1000.0,(done-prep_done)/1000.0,(long long)((done-captured_at_us)/1000));
        last_report=done;
    }
    camera_display_show_digit(frame,input,&region,accepted?best:-1,score,stable);
    if(atomic_exchange(&dump_requested,false)) dump_frame(frame);
}
