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

// Library for using ESP32-S3 hardware timer (general purpose timer, GPTIMER) more easily. 
//  It provides a C wrapper for the ESP-IDF GPTIMER driver

#ifndef UB_ESP32_S3_TIMERHW_H
#define UB_ESP32_S3_TIMERHW_H

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gptimer.h>
#include <esp_log.h>

// create a timer, with the specified configuration, and return the handle through the pointer argument
esp_err_t ub_esp32_s3_timerhw_create(gptimer_clock_source_t clk_src, gptimer_count_direction_t direction, uint32_t resolution_hz, 
    gptimer_handle_t *timer_handle);

// configure the timer alarm, which will trigger the alarm event when the timer count reaches the specified value. 
// The callback function will be called when the alarm event happens. The user_data pointer can be used to pass any data to the callback function.
// Note: usually the user_data pointer is used to pass a queue handle (QueueHandle_t), so that the callback can send data to the main task through the queue
esp_err_t ub_esp32_s3_timerhw_set_alarm(gptimer_handle_t timer_handle, 
    uint64_t reload_count, uint64_t alarm_tick_count, bool auto_reload, 
    gptimer_alarm_cb_t callback, void *user_data);

// enable the timer
esp_err_t ub_esp32_s3_timerhw_enable(gptimer_handle_t timer_handle);

// start the timer, the timer will start counting after this function is called
esp_err_t ub_esp32_s3_timerhw_start(gptimer_handle_t timer_handle);

// stop the timer, the timer will stop counting after this function is called
esp_err_t ub_esp32_s3_timerhw_stop(gptimer_handle_t timer_handle);

// disable the timer. Disabled timers can be reconfigured
esp_err_t ub_esp32_s3_timerhw_disable(gptimer_handle_t timer_handle);

// delete the timer and free resources
esp_err_t ub_esp32_s3_timerhw_delete(gptimer_handle_t timer_handle);

#endif /* UB_ESP32_S3_TIMERHW_H */