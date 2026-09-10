#include "tour_speech.h"
#include "xiaozhi_client.h"
#include "esp_timer.h"
#include <stdint.h>
#include <stddef.h>
#include <stdatomic.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "usb/uac_host.h"
extern const uint8_t *const tour_speech_clips[6];
extern const size_t tour_speech_lengths[6];
typedef struct { uint8_t address, interface; } connection_t;
typedef struct { int clip; unsigned generation; } request_t;
static QueueHandle_t connections, requests;
static uac_host_device_handle_t speaker;
static atomic_bool ready, disconnected, failed;
static atomic_int pending;
static atomic_uint generation;
static QueueHandle_t mic_connections;
static atomic_bool mic_lost, mic_fault;
static bool cloud_output;
static atomic_llong cloud_last;

bool tour_speech_ready(void) { return atomic_load(&ready); }
bool tour_speech_busy(void) { return atomic_load(&pending) > 0; }
bool tour_speech_failed(void) { return atomic_load(&failed); }
void tour_speech_cancel(void) { atomic_fetch_add(&generation, 1); xiaozhi_command("xz stop"); }
bool tour_speech_play(int clip)
{
    if (clip < 0 || clip >= 6 || !requests || !tour_speech_ready()) return false;
    request_t r = {clip, atomic_load(&generation)};
    atomic_fetch_add(&pending, 1);
    if (xQueueSend(requests, &r, 0) != pdTRUE) {
        atomic_fetch_sub(&pending, 1);
        return false;
    }
    xiaozhi_audio_fault(); /* Announcements preempt dialogue without blocking motors. */
    return true;
}
static void device_event(uac_host_device_handle_t h, uac_host_device_event_t e, void *arg)
{
    (void)h; (void)arg;
    if (e == UAC_HOST_DRIVER_EVENT_DISCONNECTED) {
        atomic_store(&disconnected, true);
        atomic_store(&ready, false);
    }
    if (e == UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR) {
        if(tour_speech_busy()) atomic_store(&failed,true);
        else xiaozhi_audio_fault();
    }
}
static void driver_event(uint8_t address, uint8_t interface, uac_host_driver_event_t e, void *arg)
{
    (void)arg;
    if(e==UAC_HOST_DRIVER_EVENT_RX_CONNECTED) {
        connection_t c={address,interface};
        if(mic_connections) xQueueSend(mic_connections,&c,0);
        return;
    }
    if (e != UAC_HOST_DRIVER_EVENT_TX_CONNECTED) return;
    connection_t c = {address, interface};
    if (xQueueSend(connections, &c, 0) != pdTRUE) atomic_store(&failed, true);
}
static void mic_event(uac_host_device_handle_t h,uac_host_device_event_t e,void *arg)
{
    (void)h; (void)arg;
    if(e==UAC_HOST_DRIVER_EVENT_DISCONNECTED) atomic_store(&mic_lost,true);
    if(e==UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR) atomic_store(&mic_fault,true);
}
static void microphone_task(void *arg)
{
    (void)arg;
    uac_host_device_handle_t mic=NULL;
    int16_t pcm[XZ_PCM_SAMPLES], chunk[320]; size_t used=0;
    for(;;) {
        connection_t c;
        if(atomic_exchange(&mic_lost,false) && mic) {
            uac_host_device_close(mic); mic=NULL; used=0; xiaozhi_audio_fault();
        }
        if(xQueueReceive(mic_connections,&c,0)==pdTRUE && !mic) {
            uac_host_device_config_t cfg={.addr=c.address,.iface_num=c.interface,
                .buffer_size=4096,.buffer_threshold=640,.callback=mic_event};
            esp_err_t err=uac_host_device_open(&cfg,&mic);
            uac_host_stream_config_t stream={.channels=1,.bit_resolution=16,.sample_freq=16000};
            if(err==ESP_OK) err=uac_host_device_start(mic,&stream);
            if(err!=ESP_OK) {
                if(mic) uac_host_device_close(mic);
                mic=NULL;
                ESP_LOGW("TOUR_MIC","Microphone unavailable: %s",esp_err_to_name(err));
            } else {
                uac_host_device_set_mute(mic,false);
                ESP_LOGI("TOUR_MIC","MIC_READY 16000Hz mono; continuous capture");
            }
        }
        if(atomic_exchange(&mic_fault,false)) { used=0; xiaozhi_audio_fault(); }
        /* Continuously drain RX, including during local speech: no stale audio later. */
        for(int n=0; mic && n<4; ++n) {
            uint32_t bytes=0;
            if(uac_host_device_read(mic,(uint8_t*)chunk,sizeof(chunk),&bytes,0)!=ESP_OK || !bytes) break;
            if(!xiaozhi_wants_microphone()) { used=0; continue; }
            for(size_t i=0;i<bytes/2;++i) {
                pcm[used++]=chunk[i];
                if(used==XZ_PCM_SAMPLES) { xiaozhi_push_pcm(pcm); used=0; }
            }
        }
        xiaozhi_set_microphone_ready(mic!=NULL);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
static void cloud_playback(void)
{
    if(esp_timer_get_time()-cloud_last>=300000 && !cloud_output) xiaozhi_set_output_active(false);
    static xz_playback_t frame;
    if(!speaker || !tour_speech_ready()) { xiaozhi_set_output_active(false); return; }
    if(xiaozhi_allows_playback() && xiaozhi_take_playback(&frame)) {
        esp_err_t err=ESP_OK;
        if(!cloud_output) {
            err=uac_host_device_set_mute(speaker,false);
            if(err==ESP_ERR_NOT_SUPPORTED) err=ESP_OK;
            if(err==ESP_OK) err=uac_host_device_resume(speaker);
        }
        if(err==ESP_OK) err=uac_host_device_write(speaker,(uint8_t*)frame.pcm,frame.samples*2,pdMS_TO_TICKS(150));
        cloud_output=true; cloud_last=esp_timer_get_time();
        if(err!=ESP_OK) xiaozhi_audio_fault(); /* Dialogue error does not latch tour failure. */
    }
    if(cloud_output && (!xiaozhi_allows_playback() || esp_timer_get_time()-cloud_last>200000)) {
        uac_host_device_suspend(speaker); cloud_output=false;
        cloud_last=esp_timer_get_time();
    }
}
static void open_speaker(connection_t c)
{
    if (speaker) return;
    uac_host_device_config_t cfg = {
        .addr=c.address, .iface_num=c.interface,
        .buffer_size=4096, .buffer_threshold=640,
        .callback=device_event,
    };
    esp_err_t err = uac_host_device_open(&cfg, &speaker);
    if (err == ESP_OK) {
        uac_host_stream_config_t stream = {
            .channels=1, .bit_resolution=16, .sample_freq=16000,
            .flags=FLAG_STREAM_SUSPEND_AFTER_START,
        };
        err = uac_host_device_start(speaker, &stream);
    }
    if (err != ESP_OK) {
        if (speaker) uac_host_device_close(speaker);
        speaker=NULL;
        atomic_store(&failed,true);
        ESP_LOGE("TOUR_AUDIO", "speaker open failed: %s", esp_err_to_name(err));
        return;
    }
    esp_err_t volume_error=uac_host_device_set_volume(speaker,100);
    ESP_LOGI("TOUR_AUDIO","volume=100%% result=%s",esp_err_to_name(volume_error));
    atomic_store(&disconnected,false);
    atomic_store(&failed,false);
    atomic_store(&ready,true);
    ESP_LOGI("TOUR_AUDIO", "SPEAKER_READY 16000Hz mono 16bit iface=%u",c.interface);
}
static void play(request_t r)
{
    if (!speaker || !tour_speech_ready() || r.generation != atomic_load(&generation)) return;
    esp_err_t err = uac_host_device_set_mute(speaker,false);
    if (err == ESP_ERR_NOT_SUPPORTED) err=ESP_OK;
    if (err == ESP_OK) err=uac_host_device_resume(speaker);
    size_t offset=0;
    int16_t pcm[320]; /* 20ms at 16kHz; 8kHz stored samples duplicated. */
    while (err == ESP_OK && offset < tour_speech_lengths[r.clip] &&
           !atomic_load(&disconnected) && !atomic_load(&failed) &&
           r.generation == atomic_load(&generation)) {
        size_t n=tour_speech_lengths[r.clip]-offset;
        if (n>160) n=160;
        for(size_t i=0;i<n;++i) {
            int value=((int)tour_speech_clips[r.clip][offset+i]-128)*384;
            int16_t sample=value>32767?32767:value<-32768?-32768:(int16_t)value;
            pcm[2*i]=pcm[2*i+1]=sample;
        }
        err=uac_host_device_write(speaker,(uint8_t*)pcm,n*4,pdMS_TO_TICKS(150));
        offset+=n;
    }
    /* Drain the bounded 4096-byte ring buffer before suspending the stream. */
    if (err == ESP_OK && r.generation == atomic_load(&generation) && !atomic_load(&disconnected))
        vTaskDelay(pdMS_TO_TICKS(160));
    if (!atomic_load(&disconnected)) uac_host_device_suspend(speaker);
    if (err != ESP_OK || atomic_load(&disconnected)) atomic_store(&failed,true);
    ESP_LOGI("TOUR_AUDIO", "clip=%d submitted=%u/%u error=%s",r.clip,
             (unsigned)offset,(unsigned)tour_speech_lengths[r.clip],esp_err_to_name(err));
}
static void audio_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (atomic_load(&disconnected) && speaker) {
            uac_host_device_close(speaker); speaker=NULL; cloud_output=false;
        }
        connection_t c;
        while(xQueueReceive(connections,&c,0)==pdTRUE) open_speaker(c);
        request_t r;
        if(xQueueReceive(requests,&r,pdMS_TO_TICKS(20))==pdTRUE) {
            if(cloud_output && speaker) {
                uac_host_device_suspend(speaker); cloud_output=false;
            }
            xiaozhi_set_output_active(true);
            play(r);
            cloud_last=esp_timer_get_time();
            atomic_fetch_sub(&pending,1);
        } else cloud_playback();
    }
}
esp_err_t tour_speech_init(void)
{
    connections=xQueueCreate(4,sizeof(connection_t));
    requests=xQueueCreate(8,sizeof(request_t));
    mic_connections=xQueueCreate(4,sizeof(connection_t));
    if(!connections || !requests || !mic_connections) return ESP_ERR_NO_MEM;
    uac_host_driver_config_t cfg = {
        .create_background_task=true,.task_priority=5,.stack_size=4096,
        .core_id=0,.callback=driver_event,
    };
    esp_err_t err=uac_host_install(&cfg);
    if(err!=ESP_OK) return err;
    if(xTaskCreatePinnedToCore(microphone_task,"tour_mic",6144,NULL,2,NULL,0)!=pdPASS)
        ESP_LOGW("TOUR_MIC","Microphone task unavailable; tour still available");
    return xTaskCreate(audio_task,"tour_speech",4096,NULL,3,NULL)==pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
