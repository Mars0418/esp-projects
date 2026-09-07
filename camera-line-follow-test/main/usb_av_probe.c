#include "usb_av_probe.h"

#include <stdbool.h>
#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_host.h"

static const char *TAG = "USB_AV_PROBE";
static usb_host_client_handle_t s_client;
/* Accessed only by the probe task, including its host-event callbacks. */
static bool s_pending[128];

static unsigned le16(const uint8_t *p)
{
    return p[0] | ((unsigned)p[1] << 8);
}

static unsigned le24(const uint8_t *p)
{
    return p[0] | ((unsigned)p[1] << 8) | ((unsigned)p[2] << 16);
}

static void inspect_device(uint8_t address)
{
    usb_device_handle_t dev;
    esp_err_t error = usb_host_device_open(s_client, address, &dev);
    if (error != ESP_OK) {
        ESP_LOGW(TAG, "open address=%u: %s", address, esp_err_to_name(error));
        return;
    }
    const usb_device_desc_t *device_desc;
    if (usb_host_get_device_descriptor(dev, &device_desc) == ESP_OK)
        ESP_LOGI(TAG, "DEVICE address=%u VID=%04x PID=%04x", address,
                 device_desc->idVendor, device_desc->idProduct);

    const usb_config_desc_t *config;
    if (usb_host_get_active_config_descriptor(dev, &config) == ESP_OK) {
        const uint8_t *bytes = (const uint8_t *)config;
        size_t total = config->wTotalLength;
        unsigned cls = 0, sub = 0, protocol = 0, intf = 0, alt = 0;
        bool mic = false, speaker = false, video = false;
        ESP_LOGI(TAG, "CONFIG bytes=%u interfaces=%u", (unsigned)total,
                 config->bNumInterfaces);
        for (size_t pos = 0; pos + 2 <= total;) {
            const uint8_t *d = bytes + pos;
            size_t len = d[0];
            if (len < 2 || len > total - pos) {
                ESP_LOGW(TAG, "Malformed descriptor offset=%u", (unsigned)pos);
                break;
            }
            if (d[1] == 4 && len >= 9) {
                intf = d[2]; alt = d[3]; cls = d[5]; sub = d[6]; protocol = d[7];
                video |= cls == 14;
                ESP_LOGI(TAG, "INTERFACE number=%u alt=%u endpoints=%u class=%02x subclass=%02x protocol=%02x",
                         intf, alt, d[4], cls, sub, protocol);
            } else if (d[1] == 5 && len >= 7 && cls == 1 && sub == 2) {
                unsigned addr = d[2], attr = d[3];
                /* Exclude feedback endpoints: usage bits 5:4 == 1. */
                bool data = (attr & 3) == 1 && (attr & 0x30) != 0x10;
                mic |= data && (addr & 0x80);
                speaker |= data && !(addr & 0x80);
                ESP_LOGI(TAG, "AUDIO_EP interface=%u alt=%u address=0x%02x direction=%s attributes=0x%02x max_packet=%u interval=%u",
                         intf, alt, addr, addr & 0x80 ? "IN_MIC" : "OUT_SPEAKER",
                         attr, le16(d + 4), d[6]);
            } else if (d[1] == 0x24 && cls == 1 && len >= 3) {
                ESP_LOG_BUFFER_HEX_LEVEL(TAG, d, len, ESP_LOG_INFO);
                if (sub == 2 && protocol == 0 && d[2] == 2 && len >= 8 && d[3] == 1) {
                    ESP_LOGI(TAG, "UAC1_FORMAT interface=%u alt=%u channels=%u subframe_bytes=%u bits=%u rate_count=%u",
                             intf, alt, d[4], d[5], d[6], d[7]);
                    unsigned count = d[7] ? d[7] : 2;
                    if (len >= 8 + count * 3) {
                        for (unsigned r = 0; r < count; ++r)
                            ESP_LOGI(TAG, "AUDIO_RATE interface=%u alt=%u index=%u hz=%u continuous=%u",
                                     intf, alt, r, le24(d + 8 + r * 3), d[7] == 0);
                    }
                }
            }
            pos += len;
        }
        ESP_LOGI(TAG, "DESCRIPTOR_CAPABILITIES video=%d microphone=%d speaker=%d (not a capture/playback test)",
                 video, mic, speaker);
    } else {
        ESP_LOGW(TAG, "Cannot read configuration address=%u", address);
    }
    error = usb_host_device_close(s_client, dev);
    if (error != ESP_OK) ESP_LOGW(TAG, "close: %s", esp_err_to_name(error));
}

static void on_event(const usb_host_client_event_msg_t *event, void *arg)
{
    (void)arg;
    if (event->event == USB_HOST_CLIENT_EVENT_NEW_DEV && event->new_dev.address < 128)
        s_pending[event->new_dev.address] = true;
}

static void probe_task(void *arg)
{
    (void)arg;
    while (true) {
        usb_host_client_handle_events(s_client, pdMS_TO_TICKS(500));
        for (unsigned address = 1; address < 128; ++address) {
            if (s_pending[address]) {
                s_pending[address] = false;
                inspect_device(address);
            }
        }
    }
}

esp_err_t usb_av_probe_start(void)
{
    const usb_host_client_config_t config = {
        .is_synchronous = false,
        .max_num_event_msg = 8,
        .async = {.client_event_callback = on_event, .callback_arg = NULL},
    };
    esp_err_t error = usb_host_client_register(&config, &s_client);
    if (error != ESP_OK) return error;
    if (xTaskCreate(probe_task, "usb_av_probe", 4096, NULL, 2, NULL) != pdPASS) {
        usb_host_client_deregister(s_client);
        s_client = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
