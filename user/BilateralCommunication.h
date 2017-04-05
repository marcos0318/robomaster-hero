#ifndef BILATERALCOMMUNICATION_H
#define BILATERALCOMMUNICATION_H
#include "stm32f4xx_dma.h"
#include "stm32f4xx.h"
#include "stdbool.h"
#include "pneumatic.h"
void Bilateral_Init();
extern volatile int32_t LiftingMotorPositionSetpoint[4];
extern volatile bool ONE_KEY_UP_FRONT;
extern volatile bool ONE_KEY_UP_BACK;
extern volatile bool ONE_KEY_DOWN_FRONT;
extern volatile bool ONE_KEY_DOWN_BACK;
extern volatile bool GO_ON_STAGE_ONE_KEY;
extern volatile bool GO_DOWN_STAGE_ONE_KEY;
extern volatile bool BREAK;

#endif