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

const static char *TAG = "ub_XiaomiLIDAR";

// Inits the Xiaomi LIDAR module, setting up UART and GPIO as needed
void ub_XiaomiLIDAR_init(void)
{
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
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    // Set UART2 RX pin, leave the rest as is (TX, RX, RTS, CTS)
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, CONFIG_UART2_RXD, 
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));    

    // Enable lidar motor
    esp_rom_gpio_pad_select_gpio(CONFIG_LIDAR_EN);
    ESP_ERROR_CHECK(gpio_set_direction(CONFIG_LIDAR_EN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(CONFIG_LIDAR_EN, 1));  // Set the GPIO high to enable the LIDAR motor
}

// Starts the FreeRTOS task that reads data from the Lidar over Serial
// This function should be called in a FreeRTOS task context
void ub_XiaomiLIDAR_SerialReadTask(void *arg)
{
    static int status = 0;
    static uint16_t checksum = 0;
    static XiaomiLidarFrame_t frame;
    /* A dataframe is formed by:
        + The header, which is 0xAA
        + The frame length, which is 2 bytes (MSB and LSB)
        + The protocol version, which is 1 byte (0x01 in our case)
        + The frame type, which is 1 byte (0x61 in our case)
        + The command word, which is 1 byte
        + The payload length, which is 2 bytes (MSB and LSB)
        + The payload data, which is variable length, althought maximum seems to be 65 bytes (in our case)
        + The checksum, which is 2 bytes (MSB and LSB)
        + The checksum calculation does not include the last 2 bytes of the frame (the proper checksum bytes)
    */

    bool current_frame_is_end_of_scan = false;

    uint8_t data[128];
    uint16_t reading_mark = 0;
    while (true)
    {
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT_NUM, (size_t*)&length));
        if (length > 0) 
        {
            // we've got data. We will read it byte by byte, and parse it according to the LIDAR protocol
            uart_read_bytes(UART_PORT_NUM, data, 1, 20 / portTICK_PERIOD_MS);
            uint8_t byte = data[0];
            switch (status)
            {
                case 0: // Waiting for header
                    if (byte == 0xAA)
                    {
                        frame.frameHeader = byte;
                        checksum = byte;
                        status = 1;
                    }
                    break;
                case 1: // Reading frame length (MSB)
                    frame.frameLength = byte << 8;
                    checksum += byte;
                    status = 2;
                    break;
                case 2: // Reading frame length (LSB)
                    frame.frameLength |= byte;
                    checksum += byte;
                    status = 3;
                    break;
                case 3: // Reading protocol version
                    frame.protocolVersion = byte;
                    checksum += byte;
                    if (frame.protocolVersion == 0x01) {
                        status = 4;
                    }
                    else {
                        ESP_LOGE(TAG, "ERROR: Protocol Version");
                        status = 0;
                    }
                    break;
                case 4: // Reading frame type
                    frame.frameType = byte;
                    checksum += byte;
                    if (frame.frameType == 0x61) {
                        status = 5;
                    }
                    else {
                        ESP_LOGE(TAG, "ERROR: Frame Type");
                        status = 0;
                    }
                    break;  
                case 5: // Command Word
                    frame.commandWord = byte;
                    checksum += byte;
                    status = 6;
                    break;
                case 6: // Parameter Length MSB
                    frame.parameterLength = byte << 8;
                    checksum += byte;
                    status = 7;
                    break;      
                case 7: // Parameter Length LSB
                    frame.parameterLength |= byte;
                    checksum += byte;
                    if (frame.parameterLength > 1) {
                        ESP_LOGI(TAG, "parameterLength: %d", frame.parameterLength);
                        status = 0;
                    }
                    reading_mark = 0;
                    status = 8;
                    break;                                            
                case 8: // Parameters
                    frame.parameters[reading_mark++] = byte;
                    checksum += byte;
                    if (reading_mark == frame.parameterLength)
                    {
                        status = 9;
                    }
                    break;
                case 9: // Checksum MSB
                    frame.checksum = byte << 8;
                    status = 10;
                    break;
                case 10: // Checksum LSB
                    frame.checksum |= byte;
                    if (frame.checksum == checksum)
                    {
                        current_frame_is_end_of_scan = ub_XiaomiLIDAR_ProcessLidarFrame(&frame);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "ERROR: Checksum mismatch");
                    }
                    status = 0;
                    if (current_frame_is_end_of_scan)
                    {
                        vTaskDelay(10);
                    }
                    break;  
            }
            if (status < 10 && status > 0)
            {
                checksum %= 0xFFFF;
            }                  
        }
        vTaskDelay(1);
    }
}

// Processes a Xiaomi Lidar frame: extracts distances, angles and pushes them to the queue
bool ub_XiaomiLIDAR_ProcessLidarFrame(XiaomiLidarFrame_t *frame)
{
    bool current_frame_is_end_of_scan = false;
    ESP_LOGI(TAG, "Frame Type: 0x%02X, Command Word: 0x%02X, Parameter Length: %d", 
        frame->frameType, frame->commandWord, frame->parameterLength);

    return current_frame_is_end_of_scan;

}