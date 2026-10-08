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

// ------------------------------------------------------
// Microbótica, GIET, Univ. Málaga
// ------------------------------------------------------
// This library contains all the definitions and functions required to move the
// robot's differential base. 

// The library is provided EMPTY to the students, who will fill it according to 
// their criteria. It is highly advisable to build this one using the provided 'ub_MCPWM' library.

#ifndef _UB_DIFF_MOTORS_H_
#define _UB_DIFF_MOTORS_H_

// Honestly, use 'ub_MCPWM' to ease developing. 
// Once you have the bot base working, feel free to use any other solution
#include "ub_MCPWM.h" 

// ------------------------------------------------------
// Put here the #defines, types, variables, etc you will use for this library

// Differential base
typedef struct diff_base {
    motor_control_context_t left_motor_ctx;
    motor_control_context_t right_motor_ctx;
    // Fill it with all you need.
} diff_base_t;



// ------------------------------------------------------
// Functions you will need to move the robot around

// Move robot at a certain speed
void speed_command(float lineal_speed, float ang_speed);

// The rest of functions included here are up to you

#endif /* _UB_DIFF_MOTORS_H_ */