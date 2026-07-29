// Wi-Fi connection management library for ESP32-S3
#pragma once

#include "esp_err.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"

esp_err_t ub_esp32_s3_wifi_init(void);

esp_err_t ub_esp32_s3_wifi_connect(char* wifi_ssid, char* wifi_password);

esp_err_t ub_esp32_s3_wifi_disconnect(void);

esp_err_t ub_esp32_s3_wifi_deinit(void);