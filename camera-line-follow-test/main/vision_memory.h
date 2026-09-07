#pragma once
#include "sdkconfig.h"
#include "esp_heap_caps.h"

/* Wi-Fi needs internal RAM; component masks/queues can use external RAM.
 * This changes allocation only, not detector thresholds or tracking. */
#ifdef CONFIG_CAR_WIFI_DEBUG
#define VISION_WORK_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#else
#define VISION_WORK_CAPS (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#endif
