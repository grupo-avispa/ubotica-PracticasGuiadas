// Copyright (c) 2026 Juan Pedro Bandera Rubio
// Copyright (c) 2026 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UB_ESP32_WIFI_H
#define UB_ESP32_WIFI_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include <time.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_sntp.h>
#include <driver/gpio.h>

#define ESP_WIFI_SSID CONFIG_WIFI_SSID // WiFi SSID
#define ESP_WIFI_PASS CONFIG_WIFI_PASSWORD // WiFi Password
#define ESP_WIFI_MAXIMUM_RETRY (int)CONFIG_WIFI_MAXIMUM_RETRY // Maximum number of WiFi connection retries

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK

void ub_esp32_wifi_init();

#endif // UB_ESP32_WIFI_H

