#pragma once
#include "esp_err.h"
#define ESP_ERR_INVALID_SIZE 4
#define ESP_RETURN_ON_FALSE(condition, error, tag, ...) do { if (!(condition)) return (error); } while (0)
