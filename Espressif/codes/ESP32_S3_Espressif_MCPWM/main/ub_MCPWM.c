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

#include "ub_MCPWM.h"

static const char *TAG = "ub_MCPWM";

// Create a DC motor using MCPWM and configure it with the specified parameters. Returns motor handler
bdc_motor_handle_t ub_mcpwm_create_dc_motor(motor_control_context_t *motor_ctrl_ctx, 
                            gpio_num_t pwm_gpio_a, gpio_num_t pwm_gpio_b, 
                            uint32_t group_id, uint32_t pwm_freq_hz, uint32_t pwm_resolution_hz)
{
    ESP_LOGI(TAG, "Create DC motor");

    // Initialize the encoder handle to NULL. It will be set later when the encoder is created.
    motor_ctrl_ctx->pcnt_encoder = NULL; 

    // Configure the motor with the specified parameters
    bdc_motor_config_t motor_config = {
        // PWM frequency in Hz. This is the frequency at which the PWM signal will toggle.
        .pwm_freq_hz = pwm_freq_hz,
        // GPIO pins for the two PWM channels. These pins will output the PWM signals to control the motor.
        .pwma_gpio_num = pwm_gpio_a,
        .pwmb_gpio_num = pwm_gpio_b,
    };    
    bdc_motor_mcpwm_config_t mcpwm_config = {
        // group ID for the MCPWM unit. If you have multiple motors, you assign different group IDs to each motor (start from 0)
        .group_id = group_id,
        // resolution in Hz. Each tick is 1/resolution seconds. For example, if resolution is 10MHz, each tick is 0.1us.
        .resolution_hz = pwm_resolution_hz,     
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    // Store the motor handle in the provided context structure
    motor_ctrl_ctx->motor = motor;
    // Return the motor handle to the caller
    return motor;
}

// Create a H-bridge quadrature decoder using PCNT and configure it with the specified parameters.
void ub_mcpwm_create_quadrature_decoder(motor_control_context_t *motor_ctrl_ctx, 
                                    gpio_num_t encoder_gpio_a, gpio_num_t encoder_gpio_b,
                                    int16_t pcnt_high_limit, int16_t pcnt_low_limit)
{
    ESP_LOGI(TAG, "Init pcnt driver to decode rotary signal");
    pcnt_unit_config_t unit_config = {
        // maximum count value for the PCNT unit. When the count reaches this value, it will trigger a watch point event.
        .high_limit = pcnt_high_limit, 
        // minimum count value for the PCNT unit. When the count reaches this value, it will trigger a watch point event.
        .low_limit = pcnt_low_limit,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    // Set the glitch filter for the PCNT unit. 
    // This filter helps to ignore any noise or glitches in the encoder signal that are shorter than the specified duration (in nanoseconds).
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    // Configure the two channels of the PCNT unit to decode the quadrature encoder signals.
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = encoder_gpio_a,
        .level_gpio_num = encoder_gpio_b,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = encoder_gpio_b,
        .level_gpio_num = encoder_gpio_a,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    // Set the edge and level actions for both channels to correctly decode the quadrature signals.
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, pcnt_high_limit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, pcnt_low_limit));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    // Store the encoder handle in the provided context structure
    motor_ctrl_ctx->pcnt_encoder = pcnt_unit;
}

// Create a PID controller for the motor speed control and configure it with the specified parameters.
void ub_mcpwm_create_pid_controller(motor_control_context_t *motor_ctrl_ctx, 
                                    float kp, float ki, float kd,
                                    float max_output, float min_output)
{
    ESP_LOGI(TAG, "Create PID control block");

    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = kp,
        .ki = ki,
        .kd = kd,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = max_output,
        .min_output   = min_output,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    // Store the PID controller handle in the provided context structure
    motor_ctrl_ctx->pid_ctrl = pid_ctrl;
}
