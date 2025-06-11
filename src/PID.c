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

//PID controller structure.

/*struct _PID{

    double Kp;
    double Ki;
    double Kd;
    double tau;
    double Ts;
    double u;
    double umin;
    double umax;
    double Tt;
    double I;
    double D;
    double past_y;
    double past_e;
    double uv;
    double past_uv;

};*/

// Initialise controller parameters and internal variables.

void PID_init(PID* controller, double Kp, double Ki, double Kd, double tau, double umin, double umax, double Ts){

    // Parameters.

    controller->Kp = Kp;
    controller->Ki = Ki;
    controller->Kd = Kd;
    controller->tau = tau;
    controller->Ts = Ts;
    controller->u = 0.0;
    controller->umin = umin;
    controller->umax = umax;
    controller->Tt = (Ki == 0.0 || Kp == 0.0) ? 0.0 : sqrt((Kp / Ki) * (Kd / Kp));
    //controller->Tt = 1.0;

    // Internal variables.

    controller->I = 0.0;
    controller->D = 0.0;
    controller->past_y = 0.0;
    controller->past_e = 0.0;
    controller->uv = 0.0;
    controller->past_uv = 0.0;

    return;

};

// Utility function used by the setters. Updates anti-windup parameter whenever the gains are changed.

void _update_Tt(PID* controller){

    controller->Tt = (controller->Ki == 0.0 || controller->Kp == 0.0) ? 0.0 : sqrt((controller->Kp / controller->Ki) * (controller->Kd / controller->Kp));
    //controller->Tt = 1.0;

    return;

};

// Getters.

double get_Kp(PID* controller){

    return controller->Kp;

};

double get_Ki(PID* controller){

    return controller->Ki;

};

double get_Kd(PID* controller){

    return controller->Kd;

};

// Setters. Tt is also updated.

void set_Kp(PID* controller, double Kp){

    controller->Kp = Kp;
    _update_Tt(controller);

    return;

};

void set_Ki(PID* controller, double Ki){

    controller->Ki = Ki;
    _update_Tt(controller);

    return;

};

void set_Kd(PID* controller, double Kd){

    controller->Kd = Kd;
    _update_Tt(controller);

    return;

};

// Compute control action.

double compute_control_action(PID* controller, double reference, double measurement){

    double error = reference - measurement;
    double output;

    controller->u = controller->Kp * error + controller->I + controller->D;
    output = controller->u;

    if(output > controller->umax)

        output = controller->umax;

    else if(output < controller->umin)

        output = controller->umin;

    controller->past_uv = controller->uv;
    controller->uv = output - controller->u;
    controller->I = controller->I + 0.5 * controller->Ki * controller->Ts * (error + controller->past_e) + 
        0.5 * controller->Ts / controller->Tt * (controller->uv + controller->past_uv);
    controller->D = (2.0 * controller->tau - controller->Ts) / (2.0 * controller->tau + controller->Ts) * controller->D - 
        (2.0 * controller->Kd) / (2.0 * controller->tau + controller->Ts) * (measurement - controller->past_y);
    controller->past_e = error;
    controller->past_y = measurement;
    
    return output;

};

// Compute control action without updating the state.

double compute_no_update(PID* controller, double reference, double measurement){

    double error = reference - measurement;
    double output;

    controller->u = controller->Kp * error + controller->I + controller->D;
    output = controller->u;

    if(output > controller->umax)

        output = controller->umax;

    else if(output < controller->umin)

        output = controller->umin;

    return output;

};

// Update controller state.

void update_controller_state(PID* controller, double reference, double measurement, double control_action){

    double error = reference - measurement;

    controller->past_uv = controller->uv;
    controller->uv = control_action - controller->u;
    controller->I = controller->I + 0.5 * controller->Ki * controller->Ts * (error + controller->past_e) + 
        0.5 * controller->Ts / controller->Tt * (controller->uv + controller->past_uv);
    controller->D = (2.0 * controller->tau - controller->Ts) / (2.0 * controller->tau + controller->Ts) * controller->D - 
        (2.0 * controller->Kd) / (2.0 * controller->tau + controller->Ts) * (measurement - controller->past_y);
    controller->past_e = error;
    controller->past_y = measurement;

    return;

};