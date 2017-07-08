#ifndef LIFTING_MOTOR_CONTROL_H
#define LIFTING_MOTOR_CONTROL_H
#include "main.h"
#include "function_list.h"
#define READ_TIME 4								//detemine the length of the GPIO state buffer, now its 50ms, may be too long
#define POWER_BUFFER_LENGTH 20
#define ANGLE_PID_LIMIT 500
#define MOVING_BOUND_1 200
#define MOVING_BOUND_2 450
#define SPEED_SETPOINT_LIMIT 1000
#define SPEED_SETOPINT_LIMIT_INIT 900
//#define UP_SETPOINT 260000						//determined by the height of the pneumatic, where pneumatice can be put on the stage precisely
#define DOWN_SETPOINT 3500//determined by the relative height between the pneumatic and the wheels, whe wheels should be put on the stage precisely
#define MID_SETPOINT 98150
#define TOTALLY_DOWN_SETPOINT 3500
#define UP_DOWN_DISTANCE 130000
#define DANCING_MODE_UP_DOWN_DIFF 19000
#define ON_ENGINEERING_ROBOT 55000
//#define DANCING_MODE_RASING_HEIGHT 60600
#define RASING_HEIGHT_FOR_INITIALIZATION 150000
#define INIT_UP_PROTECTION_TIME 3200
#define INIT_DOWN_PROTECTION_TIME 3200

//extern u32 UP_SETPOINT;                  //FLASH_MEM[0]
//extern u32 DANCING_MODE_RASING_HEIGHT;   //FLASH_MEM[1]
extern u32 FLASH_MEM[2];

//type to be u32 so as to fit in the write_flash() function
//when to update them ?
//need to read getPositionSetpoint(), the second field of the data sent by Data_Monitor
//when to update them by flash ?

extern int32_t LiftingMotorSpeedFeedback[4];
extern float LiftingMotorPositionFeedback[4];

extern int32_t LiftingMotorSpeedSetpoint[4];
extern int32_t LiftingMotorSpeedSetpointBuffered[4];


extern volatile int32_t LiftingMotorPositionSetpoint[4];
extern int32_t LiftingMotorPositionSetpointBuffered[4];

extern int32_t LiftingMotorBias[4];		//actually its the lower limit
//extern int32_t LiftingMotorUpperLimit[4];
extern int32_t LiftingMotorPositionLimit[4];

extern volatile bool ONE_KEY_UP_FRONT;
extern volatile bool ONE_KEY_UP_BACK;
extern volatile bool ONE_KEY_DOWN_FRONT;
extern volatile bool ONE_KEY_DOWN_BACK;
extern volatile bool GO_ON_STAGE_ONE_KEY;		//become false when the process is finished, or by having received the break command
extern volatile bool GO_DOWN_STAGE_ONE_KEY;		//become false when the process is finished, or by having received the break command
extern volatile bool FRICTION_WHEEL_STATE;
extern bool GO_ON_STAGE_ONE_KEY_PREV;
extern bool GO_DOWN_STAGE_ONE_KEY_PREV;
extern volatile bool BREAK;
//extern u32 ticks_msimg_on_prev=0, ticks_msimg_down_prev=0;
extern struct fpid_control_states LiftingMotorPositionState[4];
extern float LMpos_kp;
extern float LMpos_ki;
extern float LMpos_kd;
extern uint16_t friction_wheel_setpoint;

//GPIO control

//Index indices which element in the array should I store current GPIO state
extern uint8_t LeftFrontIndex;
extern uint8_t LeftBackIndex;
extern uint8_t RightFrontIndex;
extern uint8_t RightBackIndex;
//GPIO state buffer
extern uint8_t LeftFrontState[READ_TIME];
extern uint8_t LeftBackState[READ_TIME];
extern uint8_t RightFrontState[READ_TIME];
extern uint8_t RightBackState[READ_TIME];





//PID controls

//The control of filter rate of wheels
// Structure to store PID data
extern struct pid_control_states LiftingMotorState[4];

extern int32_t LiftingMotorOutput[4];
extern int32_t kp, ki, kd;
void LiftingMotorInit();

#endif