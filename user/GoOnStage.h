#ifndef GOONSTAGE_H
#define GOONSTAGE_H
#include "Lifting_Motor_Control.h"
#include "initialization_process.h"
#include "ticks.h"
extern volatile uint8_t INIT_FLAG ;		//to be modified by initialization_process
extern volatile uint8_t DANCING_MODE_FLAG ; 	//used to catch golf, need to up down oscillate
extern uint8_t BROKEN_CABLE;
void readFeedback();
void speedProcess();
void setSetpoint();
uint8_t num_of_touch(const GPIO*);
void TIM7_Int_Init(u16 period,u16 psc);
#endif