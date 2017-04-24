#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H
#include "function_list.h"
#include "stdbool.h"
#include "hero_param.h"
/****************************************************************/
/***** Gimbal Yaw Control (Position loop and velocity loop) *****/
/****************************************************************/

//position control
extern float gimbalPositionSetpoint;// prevGimbalPositionSetpoint = 0;
extern float bufferedGimbalPositionSetpoint;
extern float gimbalPositionFeedback;
extern bool isGimbalPositionSetpointIncrease;
extern struct fpid_control_states gimbalPositionState;
extern int32_t yawPosMultiplier;		//DBUS mouse yaw control

//velocity control
extern struct inc_pid_states gimbalSpeedMoveState;// gimbalSpeedStaticState;
extern int32_t gimbalSpeedSetpoint;
extern int32_t gimbalSpeedMoveOutput;
extern int32_t outsideLimit;


/********************************/
/***** Gimbal Pitch Control *****/
/********************************/

//position control
extern float pitchPositionSetpoint;// prevGimbalPositionSetpoint = 0;
extern float bufferedPitchPositionSetpoint;
extern float pitchPositionFeedback;
extern bool isPitchPositionSetpointIncrease;
extern int32_t storedPitch;
extern struct fpid_control_states pitchPositionState;

//velocity control
extern struct inc_pid_states pitchSpeedMoveState;// gimbalSpeedStaticState;
extern int32_t pitchSpeedSetpoint;
extern int32_t pitchSpeedMoveOutput;
extern int32_t pitchPosMultiplier;       //DBUS mouse pitch control

void gimbal_yaw_control();
void gimbal_pitch_control();

#endif
