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

#ifndef UB_ESP32_MQTT_H
#define UB_ESP32_MQTT_H

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

#include "esp_log.h"
#include "mqtt_client.h"

#include "cJSON.h"

#define BROKER_URL CONFIG_BROKER_URL
#define BROKER_PORT CONFIG_BROKER_PORT
#define MQTT_USERNAME CONFIG_MQTT_USERNAME
#define MQTT_PASSWORD CONFIG_MQTT_PASSWORD

#define MQTT_TOPIC CONFIG_MQTT_TOPIC // Topic to subscribe to and publish messages

// Structure to hold robot data. This is an EXAMPLE structure, you can modify it according to your needs.
struct robot_data {
    uint8_t  robot_id;
    double pos_x;
    double pos_y;
    double speed;
    double heading;
};

// init MQTT client and connect to the broker. 
//      This function should be called before any publish or subscribe operations.
void ub_esp32_mqtt_init(void);

// publish a message to the MQTT broker. 
//      Returns the message ID if successful, or -1 if the client is not initialized.
int ub_esp32_mqtt_publish(const char *message, int qos, int retain);

// publish robot data to the MQTT broker as a serialized JSON string. 
//      Returns the message ID if successful, or -1 if the client is not initialized.
int ub_esp32_mqtt_publish_robot_data(struct robot_data *data, int qos, int retain);

#endif // UB_ESP32_MQTT_H