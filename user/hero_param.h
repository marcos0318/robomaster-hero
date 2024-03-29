#ifndef __HERO_PARAM_H
#define __HERO_PARAM_H

#include <stdint.h>
#include <stdbool.h>
#include <pid.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h> 
#include <math.h>

#define BUFFER_LENGTH 300	
#define POWER_BUFFER_LENGTH 20
#define CHASSIS_ANGULAR_VELOCITY_LIMIT 500
#define MOVING_BOUND_1 200
#define MOVING_BOUND_2 450
#define LiftingMotorSetpointLimit 31999
#define UP_SETPOINT 255000					//determined by the height of the pneumatic, where pneumatice can be put on the stage precisely
#define MID_SETPOINT 164000					//determined by the relative height between the pneumatic and the wheels, whe wheels should be put on the stage precisely
#define DOWN_SETPOINT 1000					//determined by the relative height between the pneumatic and the wheels, whe wheels should be put on the stage precisely
#define TOTALLY_DOWN_SETPOINT 1000
#define CAMERA_ARRAY_MULTIPLIER = 2430;

/***************************/
/***** PID Parameters ******/
/***************************/

//int32_t chassisAnglePID[3] = {1, 0, 1};
extern int32_t kp_chassisAngle;
extern int32_t ki_chassisAngle;
extern int32_t kd_chassisAngle;

//int32_t powerPID[3] = {10, 3, 0};
extern int32_t kp_power;
extern int32_t ki_power;
extern int32_t kd_power;

//DBUS for gimbal
extern int32_t direction;
extern int32_t upperTotal;

extern int32_t xtotal;
extern int32_t pre_xtotal;

//float gimbalPositionPID[3] = {0.5, 0.00032, 22};
extern float kp_gimbalPosition;
extern float ki_gimbalPosition;
extern float kd_gimbalPosition;

//float pitchPositionPID[3] = {0.4, 0.0003, 12};
extern float kp_pitchPosition;
extern float ki_pitchPosition;
extern float kd_pitchPosition;

//float cameraPositionPID[3] = {0.3, 0.00, 1};
extern float kp_cameraPosition;
extern float ki_cameraPosition;
extern float kd_cameraPosition;

//int32_t cameraSpeedPID[3] = {80, 4, 1};
extern int32_t kp_cameraSpeed;
extern int32_t ki_cameraSpeed;
extern int32_t kd_cameraSpeed;

//The control of filter rate of wheels 
// Structure to strore PID data 
extern struct pid_control_states states[4];
extern int32_t wheel_setpoints[4];
extern int32_t wheel_feedbacks[4];
extern int32_t wheel_outputs[4];

extern int32_t kp, ki, kd; // What is the physical meaning of this one?

//The control of angle of chasis
extern struct pid_control_states state_angle;
extern int32_t setpoint_angle;
extern int32_t feedback_angle;
extern int32_t output_angle_speed;




/*****************************/
/***** DBUS control data *****/
/*****************************/

extern int32_t speed_limitor;
extern int32_t speed_multiplier;
extern int32_t angular_speed_limitor;
extern int32_t forward_speed;
extern int32_t right_speed;
extern int32_t increment_of_angle;
extern int32_t mouse_prev;
extern float gimbalNotOutGyroOutput;
extern bool locked;   //for key F
extern bool Fprev;


/*************************/
/***** Power Control *****/
/*************************/

// NOT intend to use pid control, but still consider using close-loop control

extern int32_t filter_rate_limit;
//int32_t power_buffer[POWER_BUFFER_LENGTH];
extern float feedback_current;
extern float feedback_voltage;
extern int32_t work_target;
extern float Pr;
extern float PL;
extern float W;
extern int32_t W_int;
extern int32_t work_pid_output;
extern int32_t wheel_setpoint_coefficient;
extern struct pid_control_states work_state;


/******************************/
/*********** Camera ***********/
/******************************/

extern int32_t cameraPositionId;
extern int32_t pressCameraChangePrev;

extern float cameraPositionFeedback;
extern float cameraPositionSetpoint;
extern float cameraPositionOutput;

extern struct fpid_control_states cameraPositionState;

extern int32_t cameraSpeedSetpoint;
extern int32_t cameraSpeedFeedback;
extern int32_t cameraSpeedOutput;

extern struct pid_control_states cameraSpeedState;
extern float cameraArray[6];


/****************/
/***** Rune *****/
/****************/

extern int32_t isRuneMode;
extern int32_t lastIsRuneMode;

extern int32_t currentLeft;
extern int32_t lastLeft;


/*****************/
/***** Debug *****/
/*****************/

extern int32_t error;
extern int32_t spSp; 
extern int32_t Cerror;


/* Upper Pneumatic State */


/* Initialize Parameters */
extern void heroParamInit();// Haven't implemented yet

/* To be optimized... */
extern int16_t LiftingMotorSetpoint[4];
extern int32_t buffer[4][BUFFER_LENGTH];

#endif /* __HERO_H */

