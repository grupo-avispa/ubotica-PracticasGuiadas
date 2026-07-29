/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gptimer.h>
#include <esp_log.h>
#include "ub_esp32_s3_timerhw.h"
#include "sdkconfig.h"

static const char *TAG = "gptimer_example";

// struct to store data sent from ISR to main task using a queue, 
// it can be extended as needed
typedef struct {
    uint64_t event_count;
} queue_element_t;

// ISR callback function, called when timer alarm event happens
static bool IRAM_ATTR alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;
    // Retrieve timer count value and send to queue
    queue_element_t ele = {
        .event_count = edata->count_value
    };
    xQueueSendFromISR(queue, &ele, &high_task_awoken);
    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

void app_main(void)
{
    queue_element_t ele;
    QueueHandle_t queue = xQueueCreate(10, sizeof(queue_element_t));
    if (!queue) {
        ESP_LOGE(TAG, "Creating queue failed");
        return;
    }

    ESP_LOGI(TAG, "Create timer handle");
    gptimer_handle_t gptimer = NULL;

    // create timer
    ESP_ERROR_CHECK(ub_esp32_s3_timerhw_create(GPTIMER_CLK_SRC_DEFAULT, GPTIMER_COUNT_UP, 
        1000000, // 1MHz, 1 tick=1us
        &gptimer));

    // configure timer alarm, set a callback function, and enable the timer
    ESP_ERROR_CHECK(ub_esp32_s3_timerhw_set_alarm(gptimer, 0, 2000000, true, alarm_cb, queue));

    // start timer
    ESP_ERROR_CHECK(ub_esp32_s3_timerhw_start(gptimer));

    // record 4 alarm events, then stop the timer
    int record = 4;
    while (record) {
        if (xQueueReceive(queue, &ele, pdMS_TO_TICKS(4000))) {
            ESP_LOGI(TAG, "Loop countdown: %d. Timer reloaded, tick count=%llu", record, ele.event_count);
            record--;
        } else {
            ESP_LOGW(TAG, "Missed one count event");
        }
    }
  
    // stop, disable and delete the timer
    ESP_LOGI(TAG, "Stop timer");
    ESP_ERROR_CHECK(ub_esp32_s3_timerhw_stop(gptimer));
    ESP_LOGI(TAG, "Disable timer");
    ESP_ERROR_CHECK(ub_esp32_s3_timerhw_disable(gptimer));
    // here we can reconfigure the timer and start it again if needed, or we can delete it when it's no longer used
    ESP_LOGI(TAG, "Delete timer");
    ESP_ERROR_CHECK(ub_esp32_s3_timerhw_delete(gptimer));

    // delete the queue
    vQueueDelete(queue);
}
