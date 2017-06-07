#ifndef SWITCH_BACK_STATE_H
#define SWITCH_BACK_STATE_H
#include "gimbal_control.h"
//Back_To_RUNNING_MODE: upper horizontal pneumatic and LiftingMotor timer and flag
extern volatile u8 B_RUNNING_MODE_UHPneu_LM_flag;
extern volatile u32 B_RUNNING_MODE_UHPneu_LM_timer;
extern void (*backState[])(void);
#endif