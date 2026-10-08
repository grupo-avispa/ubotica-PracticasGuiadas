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

/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-freertos-software-timers-interrupts/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  Modified for execution using ESP-IDF by: Juan Pedro Bandera Rubio (Dpto. Tecnología Electrónica, Universidad de Málaga)
   Date: June 2026
*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <driver/gpio.h>
#include "sdkconfig.h"

#define LED_PIN (gpio_num_t)CONFIG_LED_GPIO
#define timer_T_ms (unsigned long)timer_period_ms

TimerHandle_t blinkTimer = NULL;

bool ledState = false;

void BlinkCallback(TimerHandle_t xTimer) {
  ledState = !ledState;
  
  if (ledState) {
    gpio_set_level(LED_PIN, 1);
    printf("LED is ON\n");
  } else {
    gpio_set_level(LED_PIN, 0);
    printf("LED is OFF\n");
  }
}

void app_main() 
{
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    blinkTimer = xTimerCreate(
        "BlinkTimer",                              // Timer name
        timer_T_ms / portTICK_PERIOD_MS,      // Timer period
        pdTRUE,                                    // Auto-reload (periodic timer)
        NULL,                                      // Timer ID
        BlinkCallback                              // Callback function
    );
    if (blinkTimer == NULL) {
        printf("Failed to create timer!\n");
        while (1);
    }

    xTimerStart(blinkTimer, 0); // Start timer immediately
}