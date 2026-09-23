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

#ifndef _UB_MCPWM_H_
#define _UB_MCPWM_H_

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

struct motor_control_context {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
};

// Create a DC motor using MCPWM and configure it with the specified parameters.
void ub_mcpwm_create_dc_motor(struct motor_control_context *motor_ctrl_ctx, 
                             gpio_num_t pwm_gpio_a, gpio_num_t pwm_gpio_b,
                             uint32_t group_id, uint32_t pwm_freq_hz, uint32_t pwm_resolution_hz);

// Create a H-bridge quadrature decoder using PCNT and configure it with the specified parameters.
void ub_mcpwm_create_quadrature_decoder(struct motor_control_context *motor_ctrl_ctx, 
                                       gpio_num_t encoder_gpio_a, gpio_num_t encoder_gpio_b,
                                       int16_t pcnt_high_limit, int16_t pcnt_low_limit);

// Create a PID controller for the motor speed control and configure it with the specified parameters.
void ub_mcpwm_create_pid_controller(struct motor_control_context *motor_ctrl_ctx, 
                                    float kp, float ki, float kd,
                                    float max_output, float min_output);                                       

#endif /* _UB_MCPWM_H_ */