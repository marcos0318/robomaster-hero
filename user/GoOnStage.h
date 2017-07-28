#ifndef GOONSTAGE_H
#define GOONSTAGE_H
#include "Lifting_Motor_Control.h"
#include "initialization_process.h"
#include "ticks.h"
#include "param.h"
extern volatile uint8_t INIT_FLAG ;		//to be modified by initialization_process
extern volatile uint8_t DANCING_MODE_FLAG ; 	//used to catch golf, need to up down oscillate
extern uint8_t BROKEN_CABLE;
extern volatile u32 TIM_7_counter;
extern volatile uint8_t CAN2BrokenLine;
extern volatile uint8_t INIT_protection_up_stop_flag;
extern volatile uint8_t INIT_protection_down_stop_flag;
extern volatile uint8_t INIT_protection_up_begin_flag;
extern volatile uint8_t INIT_protection_down_begin_flag;
extern volatile u32 INIT_protection_timer_begin;
extern volatile u32 INIT_protection_timer_reach;
extern volatile u32 INIT_protection_timer_down;

extern volatile uint32_t CAN2BrokenLineCounter;
extern volatile uint32_t Wheel1BrokenLineCounter;
extern volatile uint32_t Wheel2BrokenLineCounter;
extern volatile uint32_t Wheel3BrokenLineCounter;
extern volatile uint32_t Wheel4BrokenLineCounter;

void readFeedback();
void speedProcess();
void setSetpoint();
uint8_t num_of_touch(const GPIO*);
void TIM7_Int_Init(u16 period,u16 psc);
#endif