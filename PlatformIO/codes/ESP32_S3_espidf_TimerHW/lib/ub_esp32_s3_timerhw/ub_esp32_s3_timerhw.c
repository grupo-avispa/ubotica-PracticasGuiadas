/* 
 * ub_esp32_s3_timerhw.c
 *
 * Library for using ESP32-S3 hardware timer (general purpose timer, GPTIMER) more easily. 
 * It provides a C wrapper for the ESP-IDF GPTIMER driver
 *
 *  Created on: 2024-06-01
 *  Author: Juan Pedro Bandera Rubio, Dpto. de Tecnología Electrónica, Universidad de Málaga
 *  License: TODO!!!
*/

#include "ub_esp32_s3_timerhw.h"

// create a timer, with the specified configuration, and return the handle through the pointer argument
esp_err_t ub_esp32_s3_timerhw_create(gptimer_clock_source_t clk_src, gptimer_count_direction_t direction, uint32_t resolution_hz,
    gptimer_handle_t *timer_handle)
{
    gptimer_config_t config = {
        .clk_src = clk_src,                 // GPTIMER_CLK_SRC_DEFAULT is the default clock source, which is APB_CLK (80MHz) for now
        .direction = direction,             // GPTIMER_COUNT_UP to count up
        .resolution_hz = resolution_hz,     // resolution in Hz, e.g. 1000000 for 1MHz, which means 1 tick = 1us
        // .intr_priority = intr_priority,     // interrupt priority, if set to 0, the driver will try to allocate an interrupt with a relative low priority (1,2,3)
        // .flags = {
        //     .intr_shared = 1,               // check the manual if you want to mess with this
        //     .allow_pd = 1                   // check the manual if you want to mess with this
        // }
    };
    // Create a timer group 0, timer 0, and get the handle
    return gptimer_new_timer(&config, timer_handle);    
}

// configure the timer alarm, which will trigger the alarm event when the timer count reaches the specified value. 
// The callback function will be called when the alarm event happens. The user_data pointer can be used to pass any data to the callback function.
// Note: usually the user_data pointer is used to pass a queue handle (QueueHandle_t), so that the callback can send data to the main task through the queue
esp_err_t ub_esp32_s3_timerhw_set_alarm(gptimer_handle_t timer_handle, 
    uint64_t reload_count, uint64_t alarm_tick_count, bool auto_reload, 
    gptimer_alarm_cb_t callback, void *user_data)
{
    // set a callback function
    gptimer_event_callbacks_t cbs = {
        .on_alarm = callback,
    };
    esp_err_t err = gptimer_register_event_callbacks(timer_handle, &cbs, user_data);
    if (err != ESP_OK) 
        return err;
    
    // enable timer
    err = gptimer_enable(timer_handle);
    if (err != ESP_OK)
        return err;

    // configure alarm
    gptimer_alarm_config_t alarm_config = {
        .reload_count = reload_count,                   // not used when auto_reload_on_alarm is set to true
        .alarm_count = alarm_tick_count,                // alarm period in ticks
        .flags.auto_reload_on_alarm = auto_reload,      // set auto-reload when alarm event happens
    };
    // set alarm action for GPTimer
    return gptimer_set_alarm_action(timer_handle, &alarm_config);
}

// enable the timer
esp_err_t ub_esp32_s3_timerhw_enable(gptimer_handle_t timer_handle) {
    return gptimer_enable(timer_handle);
}

// start the timer, the timer will start counting after this function is called
esp_err_t ub_esp32_s3_timerhw_start(gptimer_handle_t timer_handle){
    return gptimer_start(timer_handle);
}

// stop the timer, the timer will stop counting after this function is called
esp_err_t ub_esp32_s3_timerhw_stop(gptimer_handle_t timer_handle) {
    return gptimer_stop(timer_handle);
}

// disable the timer. Disabled timers can be reconfigured
esp_err_t ub_esp32_s3_timerhw_disable(gptimer_handle_t timer_handle) {
    return gptimer_disable(timer_handle);
}

// delete the timer and free resources
esp_err_t ub_esp32_s3_timerhw_delete(gptimer_handle_t timer_handle) {
    return gptimer_del_timer(timer_handle);
}