#include "Lifting_Motor_Control.h"
int32_t LiftingMotorSpeedFeedback[4]={0};
float LiftingMotorPositionFeedback[4]={0};
volatile int32_t LiftingMotorPositionSetpoint[4]={0};
int32_t LiftingMotorSpeedSetpoint[4]={0};
int32_t LiftingMotorSpeedSetpointBuffered[4]={0};
int32_t LiftingMotorBias[4] = {0};		//actually its the lower limit
int32_t LiftingMotorUpperLimit[4] = {UP_SETPOINT, UP_SETPOINT, UP_SETPOINT, UP_SETPOINT};
volatile bool ONE_KEY_UP_FRONT;
volatile bool ONE_KEY_UP_BACK;
volatile bool ONE_KEY_DOWN_FRONT;
volatile bool ONE_KEY_DOWN_BACK;
volatile bool GO_ON_STAGE_ONE_KEY;		//become false when the process is finished, or by having received the break command
volatile bool GO_DOWN_STAGE_ONE_KEY;		//become false when the process is finished, or by having received the break command
volatile bool FRICTION_WHEEL_STATE;
bool GO_ON_STAGE_ONE_KEY_PREV;
bool GO_DOWN_STAGE_ONE_KEY_PREV;
volatile bool BREAK;
//extern u32 ticks_msimg_on_prev=0, ticks_msimg_down_prev=0;
struct fpid_control_states LiftingMotorPositionState[4];
float LMpos_kp;
float LMpos_ki;
float LMpos_kd;
uint16_t friction_wheel_setpoint;

//PID controls

//The control of filter rate of wheels
// Structure to strore PID data
struct pid_control_states LiftingMotorState[4];

int32_t LiftingMotorOutput[4];
int32_t kp, ki, kd;
void LiftingMotorInit(){
	GO_ON_STAGE_ONE_KEY=false;		//become false when the process is finished, or by having received the break command
	GO_DOWN_STAGE_ONE_KEY=false;		//become false when the process is finished, or by having received the break command
	FRICTION_WHEEL_STATE=0;
	GO_ON_STAGE_ONE_KEY_PREV=false;
	GO_DOWN_STAGE_ONE_KEY_PREV=false;
	BREAK=false;
//	ticks_msimg_on_prev=0, ticks_msimg_down_prev=0;
	for(uint8_t i=0;i<4;i++){
		LiftingMotorPositionState[i].cummulated_error=0;
		LiftingMotorPositionState[i].current_error=0;
		LiftingMotorPositionState[i].last_error=0;
	}
	LMpos_kp = 0.3;
	LMpos_ki = 0.0005;
	LMpos_kd = 20;
	friction_wheel_setpoint=0;

//PID controls

//The control of filter rate of wheels
// Structure to strore PID data
	for(uint8_t i=0;i<4;i++){
		LiftingMotorState[i].cummulated_error=0;
		LiftingMotorState[i].current_error=0;
		LiftingMotorState[i].last_error=0;
	}


	for(uint8_t i=0;i<4;i++){
		LiftingMotorOutput[i]=0;
	}
	kp = 80;
	ki = 4;
	kd = 1;
}