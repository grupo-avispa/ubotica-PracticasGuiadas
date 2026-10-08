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

#include "ub_XiaomiLIDAR.h"

// Inits the Xiaomi LIDAR module, setting up UART and GPIO as needed
void ub_XiaomiLIDAR_init(void)
{
    // Enable lidar motor
    esp_rom_gpio_pad_select_gpio(CONFIG_LIDAR_EN);
    ESP_ERROR_CHECK(gpio_set_direction(CONFIG_LIDAR_EN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(CONFIG_LIDAR_EN, 1));  // Set the GPIO high to enable the LIDAR motor

    // Configure UART2 for LIDAR communication
    uart_config_t uart_config = {
        .baud_rate = CONFIG_UART2_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    // Set UART2 RX pin, leave the rest as is (TX, RX, RTS, CTS)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_PIN_NO_CHANGE, CONFIG_UART2_RXD, 
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));    

}

// Starts the FreeRTOS task that reads data from the Lidar over Serial
void SerialReadTask(void *pvParameters)
{
    // Implement the task that reads data from the Lidar over Serial
    // This function should be called in a FreeRTOS task context
}