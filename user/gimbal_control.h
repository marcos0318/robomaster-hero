#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H
#include "function_list.h"
#include "stdbool.h"
#include "hero_param.h"
#include "Driver_Gun.h"
#include "slave_communication.h"
#include "switch_back_state.h"
/****************************************************************/
/***** Gimbal Yaw Control (Position loop and velocity loop) *****/
/****************************************************************/

extern volatile u32 TIM_7_Counter;

extern bool is_qe_turning;


//position control
extern float gimbalPositionSetpoint;// prevGimbalPositionSetpoint = 0;
extern float bufferedGimbalPositionSetpoint;
extern float gimbalPositionFeedback;
extern bool isGimbalPositionSetpointIncrease;
extern struct fpid_control_states gimbalPositionState;
extern int32_t yawPosMultiplier;		//DBUS mouse yaw control

//velocity control
//extern struct inc_pid_states gimbalSpeedMoveState;// gimbalSpeedStaticState;
extern float gimbalSpeedSetpoint;
extern float gimbalSpeedMoveOutput;
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
extern u8 LeftJoystick;
//velocity control

extern float pitchSpeedSetpoint;
extern float pitchSpeedFeedback;
extern float pitchSpeedMoveOutput;

extern int32_t pitchPosMultiplier;       //DBUS mouse pitch control

void gimbal_yaw_control();
void gimbal_pitch_control();
void camera_position_control();
void TIM7_Int_Init(u16 period,u16 psc);
void keyboard_mouse_control();

#endif
