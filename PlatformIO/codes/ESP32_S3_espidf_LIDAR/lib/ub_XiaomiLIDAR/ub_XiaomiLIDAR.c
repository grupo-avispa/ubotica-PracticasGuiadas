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

    // Create FreeRTOS queue for communication between tasks
    lidar_data_queue = xQueueCreate(600, sizeof(uint16_t));
    
    // Enable LIDAR PWM control
    uint32_t pwm_duty = (CONFIG_LIDAR_PWM_DUTY * ((1 << LEDC_TIMER_13_BIT) - 1)) / 100; // Convert percentage to duty value 
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT, // Set duty resolution to 13 bits
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = CONFIG_LIDAR_PWM_FREQ,  // Set output frequency
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .gpio_num       = CONFIG_LIDAR_PWM, // GPIO pin for PWM output
        .duty           = pwm_duty, // Set duty cycle
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    // Set duty cycle
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_duty));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
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
                    if (byte == XIAOMI_LIDAR_FRAME_HEADER)
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
                    if (frame.protocolVersion == XIAOMI_LIDAR_PROTOCOL_VERSION) {
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
                    if (frame.frameType == XIAOMI_LIDAR_FRAME_TYPE) {
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
                        XiaomiLidarFrame_t frame_completed = frame; // Create a copy of the frame to pass to the processing function
                        current_frame_is_end_of_scan = ub_XiaomiLIDAR_ProcessLidarFrame(frame_completed);
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
bool ub_XiaomiLIDAR_ProcessLidarFrame(XiaomiLidarFrame_t frame)
{
    static uint16_t last_start_angle = 0;

    float rotation_speed_scale = 3.0f;  // Xiaomi LIDAR conversion factor from speed to RPM (0.05f * 60.0f)
    float angle_scale = 0.01f; // Xiaomi LIDAR conversion factor from angle to degrees (0.01f)
    float range_scale = 0.25f * 0.001f; // Xiaomi LIDAR conversion factor from range to meters (0.25f * 0.001f)
    bool current_frame_is_end_of_scan = false;
 
    switch (frame.commandWord)
    {
    case 0xAE: // Device Health Information
        float rpm = frame.parameters[0] * rotation_speed_scale;
        // ESP_LOGI(TAG, "Device Health Information: \n");
        // ESP_LOGI(TAG, "RPM: %f\n", rpm);
        break;
    case 0xAD: // Scan Data
        // parameters array structure:
        // [0] - Rotation speed (need to apply the rotation_speed_scale factor to get RPM)

        ESP_LOGI(TAG, "Scan Data:");

        // Get the minimum distance and corresponding angle from the parameters
        float min_distance = range_scale * (float)(XIAOMI_LIDAR_END_OF_SCAN); // Initialize to maximum possible distance
        uint16_t min_distance_angle = 0;
        uint16_t start_angle_read = (frame.parameters[3] << 8) | frame.parameters[4];
        float start_angle = angle_scale * (float)start_angle_read; // Convert to degrees
        uint16_t min_distance_quality = 0; // Quality value corresponding to the minimum distance

        int sample_cnt = (int)(((frame.parameterLength - 5) / 3));
        ESP_LOGI(TAG, "all measures:");
        for ( int i = 0; i < sample_cnt; i++ )
        {
            uint16_t quality = (frame.parameters[5 + (i * 3)]); // Quality value, not used in this example
            uint16_t distance_read = (frame.parameters[5 + (i * 3) + 1] << 8) | frame.parameters[5 + (i * 3) + 2];
            float distance = range_scale * (float)distance_read; // Convert to meters
            float angle = start_angle + (i * 360.0f / sample_cnt); // Calculate the angle corresponding to this sample
            if (angle >= 360.0f) {
                angle -= 360.0f; // Wrap around if angle exceeds 360 degrees
            }
            
            ESP_LOGI(TAG, "Sample %d: Distance: %f m, Angle: %f degrees, Quality: %d", i, distance, angle, quality);

            if ( (distance < min_distance)&&(quality > 0) )
            {
                min_distance = distance;
                min_distance_angle = angle; // Use the calculated angle
                min_distance_quality = quality;
            }
        }
        
        // TODO: Need zero-crossing detection to create meaningful frames!!!
        // A frame is NOT a complete 360-degree scan, it is just a packet of data. We need to detect when the LIDAR crosses the 0-degree mark to know when a full scan has been completed.



        ESP_LOGI(TAG, "RPM: %f", frame.parameters[0] * rotation_speed_scale);
        ESP_LOGI(TAG, "Start Angle: %f degrees , sample count: %d", start_angle, sample_cnt);
        ESP_LOGI(TAG, "Minimum Distance: %f m at Angle: %d degrees, with quality %d\n", 
            min_distance, min_distance_angle, min_distance_quality);
        
        // int sample_cnt = (int)(((frame.parameterLength - 5) / 3));
        // for (int i = 0; i < sample_cnt; i++)
        // {
        //     uint16_t distance = (frame.parameters[5 + (i * 3) + 1] << 8) | frame.parameters[5 + (i * 3) + 2];
            
        //     // Any distance can be end of scan marker evasion
        //     if (distance == XIAOMI_LIDAR_END_OF_SCAN)
        //     {
        //         distance = XIAOMI_LIDAR_END_OF_SCAN - 1;
        //     }
            
        //     // Mandamos solo la distancia a la cola
        //     if (xQueueSend(lidar_data_queue, &distance, portMAX_DELAY) != pdTRUE)
        //     {
        //         ESP_LOGE(TAG, "ERROR: Failed to send scan data to queue");
        //     }
        // }

        // // DETECCIÓN DE CRUCE POR CERO
        // // Si el ángulo inicial de este paquete es menor que el del paquete anterior,
        // // significa que el Lidar ha cruzado físicamente la marca de los 0 grados.
        // if (start_angle < last_start_angle)
        // {
        //     uint16_t end_of_scan = XIAOMI_LIDAR_END_OF_SCAN;
        //     if (xQueueSend(lidar_data_queue, &end_of_scan, portMAX_DELAY) != pdTRUE)
        //     {
        //         ESP_LOGE(TAG, "ERROR: Failed to send end of scan to queue");
        //     }
        //     current_frame_is_end_of_scan = true;
        // }
        
        // // Actualizamos el ángulo para compararlo en la siguiente iteración
        // last_start_angle = start_angle;        


        break;
    default:
        ESP_LOGW(TAG, "Unknown Command Word: 0x%02X", frame.commandWord);
        break;
    }
    
    return current_frame_is_end_of_scan;
}
