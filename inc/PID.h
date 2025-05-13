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

/*********************************************************************
* 
* @file PID.h
*
* @author Miguel Loureiro
*
* @brief Function and type declarations for STM32 PID control library.
*
* @copyright Copyright (c) 2025 Miguel Loureiro
*
*********************************************************************/

#ifndef __PID_H__
#define __PID_H__

/***********************************************
* 
* @typedef PID controller structure declaration.
*
* @brief PID controller structure declaration.
*
***********************************************/
typedef struct _PID PID;

/******************************
* 
* @brief Initialise PID struct.
*
* @param[out] controller Pointer to a PID controller struct.
* @param[in] Kp Proportional gain.
* @param[in] Ki Integral gain.
* @param[in] Kd Derivative gain.
* @param[in] tau Derivative filter pole.
* @param[in] umin Minimum allowable control action.
* @param[in] umax Maximum allowable control action.
* @param[in] Ts Controller sampling time.
*
******************************/
void PID_init(PID* controller, double Kp, double Ki, double Kd, double tau, double umin, double umax, double Ts);

/**********************************************************
*
* @brief Get the proportional gain.
*
* @param[in] controller Pointer to a PID controller struct.
*
* @return Proportional gain.
*
**********************************************************/
double get_Kp(PID* controller);

/**********************************************************
*
* @brief Get the integral gain.
*
* @param[in] controller Pointer to a PID controller struct.
*
* @return Integral gain.
*
**********************************************************/
double get_Ki(PID* controller);

/**********************************************************
*
* @brief Get the derivative gain.
*
* @param[in] controller Pointer to a PID controller struct.
*
* @return Derivative gain.
*
**********************************************************/
double get_Kd(PID* controller);

/**********************************************************
*
* @brief Set the proportional gain.
*
* @param[in] controller Pointer to a PID controller struct.
*
* @param[in] Kp New proportional gain.
*
**********************************************************/
void set_Kp(PID* controller, double Kp);

/**********************************************************
*
* @brief Set the integral gain.
*
* @param[in] controller Pointer to a PID controller struct.
*
* @param[in] Kp New integral gain.
*
**********************************************************/
void set_Ki(PID* controller, double Ki);

/**********************************************************
*
* @brief Set the derivative gain.
*
* @param[in] controller Pointer to a PID controller struct.
*
* @param[in] Kp New derivative gain.
*
**********************************************************/
void set_Kd(PID* controller, double Kd);

/**********************************************************
*
* @brief Compute the next control action.
*
* @details Compute the next control action based on the 
*   latest output measurement. The controller state is
*   automatically updated. Note that this is
*   computationally more costly than computing the control
*   action and updating the state separately.
*
* @param[in] controller Pointer to a PID controller struct.
*
* @param[in] reference Output reference value.
*
* @param[in] measurement Latest output measurement.
*
* @return New control action.
*
* @see compute_no_update
*
**********************************************************/
double compute_control_action(PID* controller, double reference, double measurement);

/**********************************************************
*
* @brief Compute the next control action.
*
* @details Compute the next control action based on the 
*   latest output measurement. The controller state is
*   NOT updated. 
*
* @param[in] controller Pointer to a PID controller struct.
*
* @param[in] reference Output reference value.
*
* @param[in] measurement Latest output measurement.
*
* @return New control action.
*
* @warning If this function is used to compute the control
*   action, the update_controller_state function must be
*   used to update the controller state before computing
*   a new control action.
*
* @see update_controller_state, compute_control_action
*
* @note This function is computationally lighter
*   than compute_control_action. Using this function will
*   minimise the computations taking place between reading
*   a sensor's value and setting an output pin.
*
**********************************************************/
double compute_no_update(PID* controller, double reference, double measurement);

/**********************************************************
*
* @brief Update the controller state.
*
* @details Update the controller's integral and derivative
*   terms, as well as past error values, measurements, and
*   control actions.
*
* @param[out] controller Pointer to a PID controller struct.
*
* @param[in] reference Output reference value.
*
* @param[in] measurement Latest output measurement.
*
* @param[in] control_action Previously computed control 
*   action.
*
* @see compute_no_update
*
**********************************************************/
void update_controller_state(PID* controller, double reference, double measurement, double control_action);

#endif