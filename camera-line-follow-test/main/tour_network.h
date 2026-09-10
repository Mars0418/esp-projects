#pragma once
#include <stdbool.h>
#include "esp_err.h"
esp_err_t tour_network_init(void);
bool tour_network_command(const char *line);
