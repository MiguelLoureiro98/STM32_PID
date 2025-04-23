/***************************************************************************
*
*   Copyright 2025 Miguel Loureiro
*
*   Licensed under the Apache License, Version 2.0 (the "License");
*   you may not use this file except in compliance with the License.
*   You may obtain a copy of the License at
*
*       http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
*
***************************************************************************/

#include "PID.h"

/*
* PID controller structure.
*/
struct _PID{

    double Kp;
    double Ki;
    double Kd;
    double tau;
    double Ts;
    double umin;
    double umax;
    double past_I;
    double past_D;
    double past_y;
    double past_e;

};

/*
* Initialise controller parameters and internal variables.
*/
void PID_init(PID* controller, double Kp, double Ki, double Kd, double tau, double umin, double umax, double Ts){

    // Parameters.
    controller->Kp = Kp;
    controller->Ki = Ki;
    controller->Kd = Kd;
    controller->tau = tau;
    controller->Ts = Ts;
    controller->umin = umin;
    controller->umax = umax;

    // Internal variables.
    controller->past_I = 0.0;
    controller->past_D = 0.0;
    controller->past_y = 0.0;
    controller->past_e = 0.0;

};