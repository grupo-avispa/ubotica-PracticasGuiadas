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

// This library provides functions to interface with the Xiaomi LIDAR module installed 
// on the microbot. The concrete LIDAR is a Xiaomi LIDAR Delta-2a

#ifndef UB_XIAOMI_LIDAR_H
#define UB_XIAOMI_LIDAR_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <esp_log.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include "sdkconfig.h"

#define TEST_TXD (UART_PIN_NO_CHANGE)
#define TEST_RXD (CONFIG_UART2_RXD)
#define TEST_RTS (UART_PIN_NO_CHANGE)
#define TEST_CTS (UART_PIN_NO_CHANGE)

#define UART_PORT_NUM      (2)
#define UART_BAUD_RATE     (CONFIG_UART2_BAUD_RATE)
#define TASK_STACK_SIZE    (CONFIG_TASK_STACK_SIZE)

#define BUF_SIZE (1024)

// XIAOMI LIDAR defined constants
#define XIAOMI_LIDAR_FRAME_HEADER 0xAA
#define XIAOMI_LIDAR_PROTOCOL_VERSION 0x01
#define XIAOMI_LIDAR_FRAME_TYPE 0x61
#define XIAOMI_LIDAR_END_OF_SCAN 65535

// Data structure representing a parsed frame from the Xiaomi Lidar
typedef struct XiaomiLidarFrame {
    uint8_t frameHeader;          // Frame header (usually 0xAA)
    uint16_t frameLength;         // Total length of the frame
    uint8_t protocolVersion;      // Protocol version (typically 0x01)
    uint8_t frameType;            // Type of frame (e.g., data or health info)
    uint8_t commandWord;          // Command ID that defines the payload structure
    uint16_t parameterLength;     // Length of the parameters/payload
    uint8_t parameters[100];      // Payload data (e.g., distances, angles)
    uint16_t checksum;            // Checksum for validating the frame
} XiaomiLidarFrame_t;

static float rotation_speed_scale = 3.0f;      // Xiaomi LIDAR conversion factor from speed to RPM (0.05f * 60.0f)
static float angle_scale = 0.01f;              // Xiaomi LIDAR conversion factor from angle to degrees (0.01f)
static float range_scale = 0.25f * 0.001f;     // Xiaomi LIDAR conversion factor from range to meters (0.25f * 0.001f)
static float degrees_per_sample = 0.9f;        // Xiaomi LIDAR degrees per sample

static TaskHandle_t ub_XiaomiLIDAR_serialTaskHandle;    // Task handle for serial reading
static TaskHandle_t getClosestTaskHandle;               // Task handle for detecting the closest object
static QueueHandle_t lidar_data_queue;                  // Queue used to pass scan data between tasks

// Inits the Xiaomi LIDAR module, setting up UART and GPIO as needed
void ub_XiaomiLIDAR_init(void);

// Starts the FreeRTOS task that reads data from the Lidar over Serial
void ub_XiaomiLIDAR_SerialReadTask(void *arg);

// Starts the FreeRTOS task that processes the Lidar data to find the closest object
void ub_XiaomiLIDAR_GetClosestObjectTask(void *arg);

// Processes a Xiaomi Lidar frame: extracts distances, angles and pushes them to the queue 
bool ub_XiaomiLIDAR_ProcessLidarFrame(XiaomiLidarFrame_t frame);

#endif // UB_XIAOMI_LIDAR_H