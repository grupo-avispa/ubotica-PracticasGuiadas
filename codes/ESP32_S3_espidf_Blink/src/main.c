/* Blink Example
   Modified from original code in the Public Domain (or CC0 licensed, at your option) at: https://github.com/platformio
   Modified by: Juan Pedro Bandera Rubio (Dpto. Tecnología Electrónica, Universidad de Málaga)
   Date: June 2026
*/
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "sdkconfig.h"

// You can change the GPIO port for the blinking LED in platformio.ini by setting CONFIG_BLINK_GPIO to the GPIO number you want to use. 
// For example, to use GPIO4, set CONFIG_BLINK_GPIO=4 in platformio.ini
#define BLINK_GPIO (gpio_num_t)CONFIG_BLINK_GPIO

void blink_task(void *pvParameter)
{
    // Configure the IOMUX register for pad BLINK_GPIO
    esp_rom_gpio_pad_select_gpio(BLINK_GPIO);
    // Set the GPIO as a push/pull output
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        // Blink off (output low)
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // Blink on (output high)
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void hello_task(void *pvParameter)
{
    while(1) {
        printf("Hello world!\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(&hello_task, "hello_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}
