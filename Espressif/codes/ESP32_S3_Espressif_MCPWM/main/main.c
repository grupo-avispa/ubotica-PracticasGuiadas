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

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <driver/pulse_cnt.h>
#include <driver/gpio.h>
#include "bdc_motor.h"
#include "pid_ctrl.h"

#include "ub_MCPWM.h"

static const char *TAG = "MCPWM_example";

// Enable this config,  we will print debug formated string, which in return can be captured and parsed by Serial-Studio
#define SERIAL_STUDIO_DEBUG           CONFIG_SERIAL_STUDIO_DEBUG

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_A              (gpio_num_t)CONFIG_LEFT_BDC_MCPWM_GPIO_A
#define BDC_MCPWM_GPIO_B              (gpio_num_t)CONFIG_LEFT_BDC_MCPWM_GPIO_B

#define BDC_ENCODER_GPIO_A            (gpio_num_t)CONFIG_LEFT_BDC_ENCODER_GPIO_A
#define BDC_ENCODER_GPIO_B            (gpio_num_t)CONFIG_LEFT_BDC_ENCODER_GPIO_B
#define BDC_ENCODER_PCNT_HIGH_LIMIT   1000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -1000

#define BDC_PID_LOOP_PERIOD_MS        10   // calculate the motor speed every 10ms
#define BDC_PID_EXPECT_SPEED          40  // expected motor speed, in the pulses counted by the rotary encoder

// Callback function for the PID control loop. This function is called periodically by the timer.
static void left_pid_loop_cb(void *args)
{
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - ctx->last_pulse_count;
    ctx->last_pulse_count = cur_pulse_count;
    ctx->report_pulses = real_pulses;

    // calculate the speed error
    float error = ctx->desired_speed - real_pulses;
    float new_speed = 0;

    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    bdc_motor_set_speed(motor, (uint32_t)new_speed);
}

// Callback function for the PID control loop. This function is called periodically by the timer.
static void right_pid_loop_cb(void *args)
{
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - ctx->last_pulse_count;
    ctx->last_pulse_count = cur_pulse_count;
    ctx->report_pulses = real_pulses;

    // calculate the speed error
    float error = ctx->desired_speed - real_pulses;
    float new_speed = 0;

    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    bdc_motor_set_speed(motor, (uint32_t)new_speed);
}

void app_main(void)
{
    // -------------------------------------------
    // Create the controller for the left motor
    motor_control_context_t left_motor_ctrl_ctx;
    bdc_motor_handle_t left_motor = NULL;
    // Create DC motor for the left side
    left_motor = ub_mcpwm_create_dc_motor(&left_motor_ctrl_ctx, (gpio_num_t)CONFIG_LEFT_BDC_MCPWM_GPIO_A, (gpio_num_t)CONFIG_LEFT_BDC_MCPWM_GPIO_B,
                                          0, BDC_MCPWM_FREQ_HZ, BDC_MCPWM_TIMER_RESOLUTION_HZ);
    if (left_motor == NULL) {
        ESP_LOGE(TAG, "Failed to create left motor");
        return;
    }
    // Create quadrature decoder for the left motor
    ub_mcpwm_create_quadrature_decoder(&left_motor_ctrl_ctx, (gpio_num_t)CONFIG_LEFT_BDC_ENCODER_GPIO_A, (gpio_num_t)CONFIG_LEFT_BDC_ENCODER_GPIO_B,
                                    BDC_ENCODER_PCNT_HIGH_LIMIT, BDC_ENCODER_PCNT_LOW_LIMIT);
    // Create PID controller for the left motor
    ub_mcpwm_create_pid_controller(&left_motor_ctrl_ctx, 0.6, 0.4, 0.2, BDC_MCPWM_DUTY_TICK_MAX - 1, 0);

    // -------------------------------------------
    // Create the controller for the right motor
    motor_control_context_t right_motor_ctrl_ctx;
    bdc_motor_handle_t right_motor = NULL;
    // Create DC motor for the right side
    right_motor = ub_mcpwm_create_dc_motor(&right_motor_ctrl_ctx, (gpio_num_t)CONFIG_RIGHT_BDC_MCPWM_GPIO_A, (gpio_num_t)CONFIG_RIGHT_BDC_MCPWM_GPIO_B,
                                           1, BDC_MCPWM_FREQ_HZ, BDC_MCPWM_TIMER_RESOLUTION_HZ);
    if (right_motor == NULL) {
        ESP_LOGE(TAG, "Failed to create right motor");
        return;
    }
    // Create quadrature decoder for the right motor
    ub_mcpwm_create_quadrature_decoder(&right_motor_ctrl_ctx, (gpio_num_t)CONFIG_RIGHT_BDC_ENCODER_GPIO_A, (gpio_num_t)CONFIG_RIGHT_BDC_ENCODER_GPIO_B,
                                    BDC_ENCODER_PCNT_HIGH_LIMIT, BDC_ENCODER_PCNT_LOW_LIMIT);
    // Create PID controller for the right motor
    ub_mcpwm_create_pid_controller(&right_motor_ctrl_ctx, 0.6, 0.4, 0.2, BDC_MCPWM_DUTY_TICK_MAX - 1, 0);

    // -------------------------------------------
    // Enable the motors and start them in forward direction
    ESP_LOGI(TAG, "Enable left motor");
    ESP_ERROR_CHECK(bdc_motor_enable(left_motor));
    ESP_LOGI(TAG, "Forward left motor");
    ESP_ERROR_CHECK(bdc_motor_forward(left_motor));

    ESP_LOGI(TAG, "Enable right motor");
    ESP_ERROR_CHECK(bdc_motor_enable(right_motor));
    ESP_LOGI(TAG, "Reverse right motor");
    ESP_ERROR_CHECK(bdc_motor_reverse(right_motor));

    // -------------------------------------------
    // Create and start periodic timers for the PID control loops
    ESP_LOGI(TAG, "Create timers to do PID calculation periodically");
    esp_timer_handle_t left_pid_loop_timer = ub_mcpwm_create_pid_loop_timer(&left_motor_ctrl_ctx, 
                                    BDC_PID_LOOP_PERIOD_MS, left_pid_loop_cb, "left_pid_loop");
    esp_timer_handle_t right_pid_loop_timer = ub_mcpwm_create_pid_loop_timer(&right_motor_ctrl_ctx, 
                                    BDC_PID_LOOP_PERIOD_MS, right_pid_loop_cb, "right_pid_loop");



    // const esp_timer_create_args_t left_periodic_timer_args = {
    //     .callback = pid_loop_cb,
    //     .arg = &left_motor_ctrl_ctx,
    //     .name = "pid_loop"
    // };
    // esp_timer_handle_t left_pid_loop_timer = NULL;
    // ESP_ERROR_CHECK(esp_timer_create(&left_periodic_timer_args, &left_pid_loop_timer));

    // ESP_LOGI(TAG, "Create a timer to do PID calculation periodically");
    // const esp_timer_create_args_t right_periodic_timer_args = {
    //     .callback = right_pid_loop_cb,
    //     .arg = &right_motor_ctrl_ctx,
    //     .name = "right_pid_loop"
    // };
    // esp_timer_handle_t right_pid_loop_timer = NULL;
    // ESP_ERROR_CHECK(esp_timer_create(&right_periodic_timer_args, &right_pid_loop_timer));

    // ESP_LOGI(TAG, "Enable motor");
    // ESP_ERROR_CHECK(bdc_motor_enable(left_motor));
    // ESP_LOGI(TAG, "Forward motor");
    // ESP_ERROR_CHECK(bdc_motor_forward(left_motor));

    // // Test the right motor
    // ESP_LOGI(TAG, "Enable right motor");
    // ESP_ERROR_CHECK(bdc_motor_enable(right_motor));
    // ESP_LOGI(TAG, "Forward right motor");
    // ESP_ERROR_CHECK(bdc_motor_reverse(right_motor));

    // ESP_LOGI(TAG, "Start motor speed loop");
    // ESP_ERROR_CHECK(esp_timer_start_periodic(right_pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));
    // ESP_ERROR_CHECK(esp_timer_start_periodic(left_pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        // the following logging format is according to the requirement of serial-studio frame format
        // also see the dashboard config file `serial-studio-dashboard.json` for more information
#if SERIAL_STUDIO_DEBUG
        printf("/*%d*/\r\n", motor_ctrl_ctx.report_pulses);
#endif
    }
}