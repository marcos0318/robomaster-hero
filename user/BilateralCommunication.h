#ifndef BILATERALCOMMUNICATION_H
#define BILATERALCOMMUNICATION_H
#include "stm32f4xx_dma.h"
#include "stm32f4xx.h"
#include "stdbool.h"
#include "pneumatic.h"
#include "Lifting_Motor_Control.h"
#include "GoOnStage.h"
#include "initialization_process.h"
void Bilateral_Init();
//volatile int32_t LiftingMotorPositionSetpoint[4];
extern u32 receive_time;
extern u32 broken_time;
extern uint8_t getID();
extern int16_t getPositionSetpoint();
#endif