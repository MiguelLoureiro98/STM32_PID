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
#include <math.h>

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
    double Tt;
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
    controller->Tt = (Ki == 0.0 || Kp == 0.0) ? 0.0 : sqrt((Kp / Ki) * (Kd / Kp));

    // Internal variables.
    controller->past_I = 0.0;
    controller->past_D = 0.0;
    controller->past_y = 0.0;
    controller->past_e = 0.0;

    return;

};

double get_Kp(PID* controller){

    return controller->Kp;

};

double get_Ki(PID* controller){

    return controller->Ki;

};

double get_Kd(PID* controller){

    return controller->Kd;

};

void set_Kp(PID* controller, double Kp){

    controller->Kp = Kp;

    return;

};

void set_Ki(PID* controller, double Ki){

    controller->Ki = Ki;

    return;

};

void set_Kd(PID* controller, double Kd){

    controller->Kd = Kd;

    return;

};

double compute_control_action(PID* controller, double reference, double measurement){

    double error = reference - measurement;
    double P;
    double I;
    double D;
    double output;

    P = controller->Kp * error;
    I = controller->past_I + 0.5 * controller->Ki * controller->Ts * (error + controller->past_e);
    D = (2.0 * controller->tau - controller->Ts) / (2.0 * controller->tau + controller->Ts) * controller->past_D - 
        (2.0 * controller->Kd) / (2 * controller->tau + controller->Ts) * (measurement - controller->past_y);

    output = P + I + D;

    if(output > controller->umax)

        output = controller->umax;

    else if(output < controller-> umin)

        output = controller->umin;

    controller->past_I = I;
    controller->past_D = D;
    controller->past_e = error;
    controller->past_y = measurement;
    
    return output;

};