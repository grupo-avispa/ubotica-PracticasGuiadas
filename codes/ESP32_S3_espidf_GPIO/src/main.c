/* 
 * ESP32_ESPIDF_GPIO
 *
 * PlatformIO project using ESPIDF framework to demonstrate GPIO input and output with interrupts on ESP32.
 *
 *  For this example you will need a protoboard, a button, a resistor (220 ohms would work), a LED and some jumper wires.
 * 
 *  Created on: 2024-06-16
 *  Author: Juan Pedro Bandera Rubio, Dpto. de Tecnología Electrónica, Universidad de Málaga
 *  License: TODO!!!
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <esp_log.h>
#include <driver/gpio.h>

static const char *TAG = "gpio_example";

#define INPUT_PIN (gpio_num_t)CONFIG_INPUT_PIN // input pin. Needs internal pull-up
#define LED_PIN (gpio_num_t)CONFIG_LED_PIN // onboard LED for ESP-WROM-32
#define LED_PIN_2 (gpio_num_t)CONFIG_LED_PIN_2 // another LED output pin

int state = 0;
QueueHandle_t interruptQueue;

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interruptQueue, &pinNumber, NULL);
}

void LED_Control_Task(void *params)
{
    int pinNumber, count = 0;
    int state = 0;
    // very inefficient loop, good for demonstration purposes...
    while (true)
    {
        if (xQueueReceive(interruptQueue, &pinNumber, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "GPIO %d was pressed %d times. The state is %d", pinNumber, count++, gpio_get_level(INPUT_PIN));
            // LED_PIN will be set to the same level as INPUT_PIN, so it will turn on when the button is pressed and turn off when released
            ESP_ERROR_CHECK(gpio_set_level(LED_PIN, gpio_get_level(INPUT_PIN)));
            // LED_PIN_2 will change state each time the button is pressed, so it will toggle on and off with each press
            state = (state == 0) ? 1 : 0; // toggle state
            ESP_ERROR_CHECK(gpio_set_level(LED_PIN_2, state));
        }
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Starting GPIO example...");

    // configure GPIO pins as general purpose input/output
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    esp_rom_gpio_pad_select_gpio(LED_PIN_2);
    esp_rom_gpio_pad_select_gpio(INPUT_PIN);

    // configure outputs
    ESP_ERROR_CHECK(gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT)); // configure onboard LED pin as output
    ESP_ERROR_CHECK(gpio_set_direction(LED_PIN_2, GPIO_MODE_OUTPUT)); // configure LED_PIN_2 as output

    // configure input with pull-up and interrupt on rising edge
    ESP_ERROR_CHECK(gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_pulldown_dis(INPUT_PIN));
    ESP_ERROR_CHECK(gpio_pullup_en(INPUT_PIN));
    ESP_ERROR_CHECK(gpio_set_intr_type(INPUT_PIN, GPIO_INTR_POSEDGE));

    // create a FreeRTOS queue to handle GPIO interrupts and a task to process them
    interruptQueue = xQueueCreate(10, sizeof(int));
    xTaskCreate(LED_Control_Task, "LED_Control_Task", 2048, NULL, 1, NULL);

    // install the GPIO driver's ETS_GPIO_INTR_SOURCE ISR handler service, which allows per-pin GPIO interrupt handlers (check API)
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    // add the interrupt handler for the input pin
    ESP_ERROR_CHECK(gpio_isr_handler_add(INPUT_PIN, gpio_interrupt_handler, (void *)INPUT_PIN));
}