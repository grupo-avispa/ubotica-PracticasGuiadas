/* 
 *  ESP32_ESPIDF_ADC_DAC
 *
 *  PlatformIO project for ESP32 using the ESP-IDF framework. It demonstrates the use of ADC and PWM peripherals, 
 *  including calibration and reading analog values. 
 * 
 *  For ADC reference: https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html
 *  For PWM reference: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/ledc.html
 * 
 *  The example uses ADC in one-shot mode (https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/adc/adc_oneshot.html),
 *  that should be enough for us. If you want to use ADC in continuous mode to be faster, please check: 
 *  https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/adc/adc_continuous.html
 * 
 *  For this example you will need a protoboard, one (or better two) potenciometers, a resistor (220 ohms would work), a LED and some jumper wires.
 *  The example configure two ADC input channels for ADC1, named CHAN_A and CHAN_B. The particular ADC1 channel associated
 *  to each of these names is defined in the platformio.ini file, along with the desired attenuation value.
 *  Then, the values of these two ADC are read and printed in the console.
 *  Finally, the value of ADC CHAN_A is used to set the duty cycle of a PWM output so 
 *  a LED connected to this PWM output pin will light up with a brightness proportional to the ADC CHAN_A value.
 *
 *  Modified on: 2024-06-17 from code licenced under:
 *  SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Author: Juan Pedro Bandera Rubio, Dpto. de Tecnología Electrónica, Universidad de Málaga
 *  License: TODO!!!
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/soc_caps.h>
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/ledc.h>
#include "sdkconfig.h"

const static char *TAG = "adc_pwm_example";

//ADC1 Channels
#define ADC1_CHAN_A CONFIG_ADC1_CHAN_A
#define ADC1_CHAN_B CONFIG_ADC1_CHAN_B
#define ADC_ATTEN CONFIG_ADC_ATTEN

static int adc_raw_A; 
static int adc_raw_B;
static int voltage_A;
static int voltage_B;

// PWM output
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          CONFIG_PWM_OUTPUT // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (7373) // Set duty to 90%. (2 ** 13) * 90% = 4096
#define LEDC_CLK_SRC            LEDC_AUTO_CLK
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

/* Warning:
 * For ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C2, ESP32C6, ESP32H2 (rev < 1.2), ESP32P4 (rev < 3.0) targets,
 * when LEDC_DUTY_RES selects the maximum duty resolution (i.e. value equal to SOC_LEDC_TIMER_BIT_WIDTH),
 * 100% duty cycle is not reachable (duty cannot be set to (2 ** SOC_LEDC_TIMER_BIT_WIDTH)).
 */
static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_CLK_SRC,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void app_main(void)
{
    // init variables
    adc_raw_A = 0;
    adc_raw_B = 0;
    voltage_A = 0;
    voltage_B = 0;

    // ADC1 init. It's better to use the ADC1, because ADC2 is used by Wi-Fi and can cause conflicts.
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ADC1 channel configuration
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN_A, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN_B, &config));

    // ADC calibration init. Using the ub_esp32_adc_calib library
    bool do_calibration1_chan_A = false;
    bool do_calibration1_chan_B = false;

    // Set the LEDC peripheral configuration
    example_ledc_init();
    float percent_duty = 0.9;
    // Set duty to 90%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    // Main loop: Read ADC values and print them
    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN_A, &adc_raw_A));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN_A, adc_raw_A);
        vTaskDelay(pdMS_TO_TICKS(100)); // each channel is read with a period of 100+100 = 200 ms aprox.

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN_B, &adc_raw_B));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN_B, adc_raw_B);
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Change the duty cycle of the PWM output based on the ADC CHAN_A value
        // JP: CHECK WHEN WE HAVE THE DEVKITC!!!
        uint32_t duty_cycle = (uint32_t)(((float)adc_raw_A / 4095.0) * (float)LEDC_DUTY);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_cycle));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    }

    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
}

