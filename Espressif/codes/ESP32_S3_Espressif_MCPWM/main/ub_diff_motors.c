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

#include "ub_diff_motors.h"

// Move robot at a certain speed
void speed_command(float lineal_speed, float ang_speed)
{
    // TO DO
}

// 'Coast' a motor is to shut off the voltage. The motor stops depending on the load. 
// In our case, it's going to be a fast stop 
void coast_robot()
{
    // TO DO
}

// 'Brake' a motor is to stop it applying a certain voltage. 
// According to the documentation, it will stop the robot softly
void brake_robot()
{
    // TO DO
}
