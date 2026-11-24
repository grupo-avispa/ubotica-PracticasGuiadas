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
    lidar_data_queue = xQueueCreate(1000, sizeof(float));
    
    // Enable LIDAR PWM control
    current_duty = (CONFIG_LIDAR_PWM_DUTY * ((1 << LEDC_TIMER_13_BIT) - 1)) / 100; // Convert percentage to duty value 
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
        .duty           = current_duty, // Set duty cycle
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    // Set duty cycle
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, current_duty));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

// Starts the FreeRTOS task that reads data from the Lidar over Serial
// This function should be called in a FreeRTOS task context
void ub_XiaomiLIDAR_SerialReadTask(void *arg)
{
    static int status = 0;

    static int last_sample_count = 0;
    static float last_start_angle = 0.0f;
    static bool last_frame_is_scan_valid = false;

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
                        last_frame_is_scan_valid = false;
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
                        last_frame_is_scan_valid = false;
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
                        // frame is valid, process it
                        XiaomiLidarFrame_t frame_completed = frame; // Create a copy of the frame to pass to the processing function
                        bool is_the_end_of_scan = ub_XiaomiLIDAR_ProcessLidarFrame(frame_completed);
                        // Check end of scan
                        if (is_the_end_of_scan)
                        {
                            vTaskDelay(10); // Delay to allow the queue to process the end of scan marker
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG, "ERROR: Checksum mismatch");
                        last_frame_is_scan_valid = false;
                    }
                    status = 0;
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
    bool is_end_of_scan = false;
    static float last_start_angle = 0.0f;

    switch (frame.commandWord)
    {
    case 0xAE: // Device Health Information
        float read_rpm = frame.parameters[0] * rotation_speed_scale;
        ESP_LOGI(TAG, "Device Health Information: \n");
        ESP_LOGI(TAG, "RPM: %f\n", read_rpm);
        // Simple controller to adjust the PWM duty cycle based on the current RPM
        float error = desired_rpm - read_rpm;
        if (error > 0) {
            if (current_duty < ((1 << LEDC_TIMER_13_BIT) - 1)) {
                current_duty += 1; // Increase duty cycle to speed up (clamped to max)
            }
            else {
                current_duty = ((1 << LEDC_TIMER_13_BIT) - 1); // Clamp to max
            }
        } else if (error < 0) {
            if (current_duty > 0) {
                current_duty -= 1; // Decrease duty cycle to slow down
            }
            else {
                current_duty = 0; // Clamp to min
            }
        }
        // Update the PWM duty cycle
        ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, current_duty, 0));
        // update the current RPM variable
        current_rpm = read_rpm;
        break;
    case 0xAD: // Scan Data
        // parameters array structure:
        // TODO

        //ESP_LOGI(TAG, "Scan Data:");

        uint16_t start_angle_read = (frame.parameters[3] << 8) | frame.parameters[4];
        float start_angle = angle_scale * (float)start_angle_read; // Convert to degrees

        int sample_cnt = (int)(((frame.parameterLength - 5) / 3));
        //ESP_LOGI(TAG, "all measures:");
        for ( int i = 0; i < sample_cnt; i++ )
        {
            uint16_t quality = (frame.parameters[5 + (i * 3)]); // Quality value, not used in this example
            uint16_t distance_read = (frame.parameters[5 + (i * 3) + 1] << 8) | frame.parameters[5 + (i * 3) + 2];
            if (distance_read == XIAOMI_LIDAR_END_OF_SCAN)
            {
                distance_read = XIAOMI_LIDAR_END_OF_SCAN - 1; // Avoid using the end of scan marker as a valid distance
            }
            float distance = range_scale * (float)distance_read; // Convert to meters
            float angle = start_angle + (i * degrees_per_sample); // Calculate the angle corresponding to this sample
            
            // CHECK JP: Invert the angle
            angle = 360.0f - angle; // Invert the angle to match the expected coordinate system

            if (angle >= 360.0f) {
                angle -= 360.0f; // Wrap around if angle exceeds 360 degrees
            }

            // if quality is not 0, send the angle and distance to the queue for further processing
            if (quality > 0) {
                // Send angle and distance to the queue for further processing
                if (xQueueSend(lidar_data_queue, &angle, portMAX_DELAY) != pdTRUE) {
                    ESP_LOGE(TAG, "ERROR: Failed to send scan angle %i to queue", i);
                }
                if (xQueueSend(lidar_data_queue, &distance, portMAX_DELAY) != pdTRUE){
                    ESP_LOGE(TAG, "ERROR: Failed to send scan distance %i to queue", i);
                }
            }           
            //ESP_LOGI(TAG, "Sample %d: Distance: %f m, Angle: %f degrees, Quality: %d", i, distance, angle, quality);
            float rpm = frame.parameters[0] * rotation_speed_scale;
            ESP_LOGI(TAG, "Device Health Information: \n");
            ESP_LOGI(TAG, "RPM: %f\n", rpm);
        }
        
        // Zero-crossing detection: if the start angle of the current frame is less than the last start angle, 
        //it indicates that a full 360-degree scan has been completed
        if (start_angle < last_start_angle)
        {
            float end_of_scan = (float)XIAOMI_LIDAR_END_OF_SCAN;
            // Send the end of scan marker to the queue to indicate that a full scan has been completed
            if (xQueueSend(lidar_data_queue, &end_of_scan, portMAX_DELAY) != pdTRUE)
            {
                ESP_LOGE(TAG, "ERROR: Failed to send end of scan to queue");
            }
            is_end_of_scan = true;
        }    
        // Update the last start angle for the next frame processing
        last_start_angle = start_angle;        
        break;
    default:
        ESP_LOGW(TAG, "Unknown Command Word: 0x%02X", frame.commandWord);
        break;
    }
    
    return is_end_of_scan; // Return whether the scan was valid or not
}

// Starts the FreeRTOS task that gets the closest object based on the Lidar data
// ---  This task is intended to be the base over with the students can create a more complex task,
//      that sends the LIDAR data via microROS2 to navigate the robot, or whatever
// This function should be called in a FreeRTOS task context
void ub_XiaomiLIDAR_GetClosestObjectTask(void *arg)
{
    float samples[1000]; // Buffer to store distance samples
    float scan;
    bool end_of_scan_received = false;
    uint16_t sample_count = 0;

    while (true)
    {
        // Store all the samples as angle-distance pairs in the buffer until the end of scan marker is received
        if (xQueueReceive(lidar_data_queue, &scan, portMAX_DELAY) == pdTRUE)
        {            
            if (uxQueueMessagesWaiting(lidar_data_queue) > 590)
            {
                ESP_LOGI(TAG, "Queue size: %d\n", uxQueueMessagesWaiting(lidar_data_queue));
            }
            
            // If we receive the end of scan marker, we have a full 360º scan
            if (scan == XIAOMI_LIDAR_END_OF_SCAN)
            {
                end_of_scan_received = true;
            }
            else
            {
                // store the scan in the buffer for processing
                if (sample_count < 1000) {
                    samples[sample_count++] = scan;
                }
                else {
                    ESP_LOGW(TAG, "Sample buffer overflow, discarding data");
                }
            } 
            
            // Check if we received the end of scan signal
            if (end_of_scan_received)
            {
                // Process the collected samples to find the closest object
                float closest_distance = 100.0f; // Initialize to a large value (e.g., 100 meters)
                uint16_t closest_angle = 0;
                for (uint16_t i = 0; i < sample_count-1; i=i+2) // Increment by 2 to process angle-distance pairs
                {
                    float distance = samples[i+1]; // Distance is stored in the next index
                    float angle = samples[i]; // Angle is stored in the current index
                    if (distance < closest_distance)
                    {
                        closest_distance = distance;
                        closest_angle = angle;
                    }
                    ESP_LOGI(TAG, "Sample %d: Angle: %f degrees, Distance: %f m", i/2, angle, distance);
                }
                ESP_LOGI(TAG, "Closest Object: Angle: %d degrees, Distance: %f m\n", closest_angle, closest_distance);
                // Reset the sample count for the next scan
                sample_count = 0;                    
                end_of_scan_received = false;
            }
        }
    }
}
