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

// This example code reads data from the Xiaomi LIDAR mounted on the microbot,
// and sends the distance and angle to the closest obstacle using the serial terminal
//
// The connection to the Xiaomi LIDAR is performed using UART2 and the pins assigned
// in the robot's board. These are the default values in the SDK Config, do not change them
// unless you want to test this code without using the robot's board.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "ub_XiaomiLIDAR.h"

const static char *TAG = "lidar_example";

void app_main(void)
{
    // Inits the LIDAR module

    ub_XiaomiLIDAR_init();

    xTaskCreatePinnedToCore(ub_XiaomiLIDAR_SerialReadTask, 
        "SerialReadTask", TASK_STACK_SIZE, NULL, 1, &ub_XiaomiLIDAR_serialTaskHandle, 0);
    xTaskCreatePinnedToCore(ub_XiaomiLIDAR_GetClosestObjectTask, 
       "GetClosestObjectTask", TASK_STACK_SIZE, NULL, 2, &getClosestTaskHandle, 1);        

}