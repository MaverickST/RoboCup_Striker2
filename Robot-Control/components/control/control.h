/**
 * @file control_pid.h
 * @author Striker2
 * @brief 
 * @version 0.1
 * @date 2025-06-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __CONTROL_PID_H__
#define __CONTROL_PID_H__

#define MIN(a,b) ((a)<(b)?(a):(b)) ///< Macro to get the minimum value
#define MAX(a,b) ((a)>(b)?(a):(b)) ///< Macro to get the maximum value

typedef struct {
    float Kp; // PID Kp value
    float Ki; // PID Ki value
    float Kd; // PID Kd value
    float prev_err1; // e(k-1)
    float prev_err2; // e(k-2)
    float prev_u1; // u(k-1)
    float prev_u2; // u(k-2)
    float integral_err;  // Sum of error
    float last_output;  // PID output in last control period
    float max_output;   // PID maximum output limitation
    float min_output;   // PID minimum output limitation
    float max_integral; // PID maximum integral value limitation
    float min_integral; // PID minimum integral value limitation

    ///< Constants for the discrete PID controller
    float a2;
    float a1; 
    float a0;
    float b2;
    float b1;
    float b0;
} pid_block_t; ///< PID controller block

/**
 * @brief General Control structure
 * 
 */
typedef struct {
    pid_block_t pid; ///< PID controller block
    float dt; ///< Sampling time in seconds
    float setpoint; ///< Setpoint value
    float output; ///< Output value of thecontroller
} control_t; ///< Control PID structure

/**
 * @brief Initialize the control structure with the PID parameters and sampling time
 * 
 * @param ctrl 
 * @param dt 
 * @param pid 
 */
void control_init(control_t *ctrl, float dt, pid_block_t pid);

/**
 * @brief Calculate the PID control value
 * 
 * @param ctrl_senfusion Pointer to the control and sensor fusion structure
 * @param error Error value
 * @return float PID control value
 */
float control_calc_pid(control_t *ctrl, float error);

/**
 * @brief Calculate the PID discrete control value
 * 
 * @param ctrl_senfusion 
 * @param value speed in rad/s 
 * @return float 
 */
float control_calc_pid_z(control_t *ctrl, float value);

/**
 * @brief Set the setpoint value for the PID controller
 * 
 * @param ctrl Pointer to the control structure
 * @param setpoint Setpoint value
 */
void control_set_setpoint(control_t *ctrl, float setpoint);

#endif // __CONTROL_PID_H__