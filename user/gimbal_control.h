#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H
#include "function_list.h"
/****************************************************************/
/***** Gimbal Yaw Control (Position loop and velocity loop) *****/
/****************************************************************/

//position control
float gimbalPositionSetpoint = 0;// prevGimbalPositionSetpoint = 0;
float bufferedGimbalPositionSetpoint = 0;
float gimbalPositionFeedback = 0;
bool isGimbalPositionSetpointIncrease = true;
struct fpid_control_states gimbalPositionState = {0,0,0};
int32_t yawPosMultiplier = 3;		//DBUS mouse yaw control

//velocity control
struct inc_pid_states gimbalSpeedMoveState;// gimbalSpeedStaticState;
int32_t gimbalSpeedSetpoint = 0;
int32_t gimbalSpeedMoveOutput = 0;
int32_t outsideLimit = 670;


/********************************/
/***** Gimbal Pitch Control *****/
/********************************/

//position control
float pitchPositionSetpoint = 0;// prevGimbalPositionSetpoint = 0;
float bufferedPitchPositionSetpoint = 0;
float pitchPositionFeedback= 0;
bool isPitchPositionSetpointIncrease = true;
int32_t storedPitch = 0;
struct fpid_control_states pitchPositionState = {0,0,0};

//velocity control
struct inc_pid_states pitchSpeedMoveState;// gimbalSpeedStaticState;
int32_t pitchSpeedSetpoint = 0;
int32_t pitchSpeedMoveOutput = 0;
int32_t pitchPosMultiplier = 3;       //DBUS mouse pitch control

void gimbal_yaw_control();
void gimbal_pitch_control();

#endif