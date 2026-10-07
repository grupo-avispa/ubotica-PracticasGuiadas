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

// This example code reads data from the Xiaomi LIDAR mounted on the microbot,
// and sends the distance and angle to the closest obstacle using the serial terminal
//
// The connection to the Xiaomi LIDAR is performed using UART2 and the pins assigned
// in the robot's board. These are the default values in the SDK Config, do not change them
// unless you want to test this code without using the robot's board.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

const static char *TAG = "lidar_example";

#define TEST_TXD (UART_PIN_NO_CHANGE)
#define TEST_RXD (CONFIG_UART2_RXD)
#define TEST_RTS (UART_PIN_NO_CHANGE)
#define TEST_CTS (UART_PIN_NO_CHANGE)

#define UART_PORT_NUM      (2)
#define UART_BAUD_RATE     (CONFIG_UART2_BAUD_RATE)
#define TASK_STACK_SIZE    (CONFIG_TASK_STACK_SIZE)

#define BUF_SIZE (1024)

static void lidar_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TEST_TXD, TEST_RXD, TEST_RTS, TEST_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        uart_write_bytes(UART_PORT_NUM, (const char *) data, len);
    }
}

void app_main(void)
{
    xTaskCreate(echo_task, "lidar_task", TASK_STACK_SIZE, NULL, 10, NULL);
}