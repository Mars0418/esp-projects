#include "tour_network.h"
#include "xiaozhi_client.h"
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
static QueueHandle_t configs;
bool tour_network_command(const char *line)
{
    if(!strncmp(line,"TIME,",5)) {
        char *end=NULL;
        long long seconds=strtoll(line+5,&end,10);
        if(end!=line+5 && !*end && seconds>=1704067200LL && seconds<4102444800LL) {
            struct timeval now={.tv_sec=(time_t)seconds,.tv_usec=0};
            settimeofday(&now,NULL);
        }
        return true;
    }
    if (strncmp(line,"WIFI,",5)) return false;
    const char *ssid=line+5, *comma=strchr(ssid,',');
    if (!comma || comma-ssid<1 || comma-ssid>32 || strlen(comma+1)>63 ||
        (strlen(comma+1)>0 && strlen(comma+1)<8)) return true;
    wifi_config_t cfg={0};
    memcpy(cfg.sta.ssid,ssid,comma-ssid);
    memcpy(cfg.sta.password,comma+1,strlen(comma+1));
    cfg.sta.pmf_cfg.capable=true;
    if (configs) xQueueOverwrite(configs,&cfg);
    memset(&cfg,0,sizeof(cfg));
    return true;
}
static void network_task(void *arg)
{
    (void)arg;
    /* Never erase NVS on an initialization error: preserve existing pairing. */
    esp_err_t err=nvs_flash_init();
    if (err==ESP_OK) err=esp_netif_init();
    if (err==ESP_OK) {
        err=esp_event_loop_create_default();
        if(err==ESP_ERR_INVALID_STATE) err=ESP_OK;
    }
    if(err==ESP_OK && !esp_netif_create_default_wifi_sta()) err=ESP_ERR_NO_MEM;
    wifi_init_config_t init=WIFI_INIT_CONFIG_DEFAULT();
    if(err==ESP_OK) err=esp_wifi_init(&init);
    if(err==ESP_OK) err=esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if(err==ESP_OK) err=esp_wifi_set_mode(WIFI_MODE_STA);
    if(err==ESP_OK) err=esp_wifi_start();
    if(err!=ESP_OK) {
        ESP_LOGW("TOUR_WIFI","Optional network disabled: %s",esp_err_to_name(err));
        vTaskDelete(NULL); return;
    }
    nvs_handle_t nvs=0;
    bool have_nvs=nvs_open("car_wifi_debug",NVS_READWRITE,&nvs)==ESP_OK;
    wifi_config_t cfg={0}; size_t size=sizeof(cfg); bool configured=false;
    err=xiaozhi_init();
    if(err!=ESP_OK) ESP_LOGW("TOUR_WIFI","XiaoZhi disabled: %s",esp_err_to_name(err));
    if(have_nvs && nvs_get_blob(nvs,"sta_config",&cfg,&size)==ESP_OK && size==sizeof(cfg))
        xQueueOverwrite(configs,&cfg);
    for (;;) {
        if(xQueueReceive(configs,&cfg,pdMS_TO_TICKS(10000))==pdTRUE) {
            esp_wifi_disconnect();
            configured=esp_wifi_set_config(WIFI_IF_STA,&cfg)==ESP_OK;
            if(configured && have_nvs) {
                nvs_set_blob(nvs,"sta_config",&cfg,sizeof(cfg)); nvs_commit(nvs);
            }
            memset(&cfg,0,sizeof(cfg));
        }
        if(configured) {
            wifi_ap_record_t ap;
            if(esp_wifi_sta_get_ap_info(&ap)!=ESP_OK) esp_wifi_connect();
        }
    }
}
esp_err_t tour_network_init(void)
{
    configs=xQueueCreate(1,sizeof(wifi_config_t));
    if(!configs) return ESP_ERR_NO_MEM;
    return xTaskCreatePinnedToCore(network_task,"tour_wifi",4096,NULL,1,NULL,0)==pdPASS ? ESP_OK:ESP_ERR_NO_MEM;
}
