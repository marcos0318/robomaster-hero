#ifndef INITIALIZATION_PROCESS_H
#define INITIALIZATION_PROCESS_H


#include "gpio.h"
#include "Lifting_Motor_Control.h"

#define LeftFront &PC2
#define LeftBack &PC3
#define RightFront &PB15
#define RightBack &PB14

extern uint8_t LeftFrontExpand ;
extern uint8_t RightFrontExpand ;
extern uint8_t RightBackExpand ;
extern uint8_t LeftBackExpand ;

extern uint8_t LeftFrontReachUpper;
extern uint8_t RightFrontReachUpper;
extern uint8_t RightBackReachUpper;
extern uint8_t LeftBackReachUpper;

extern u8 ALL_TO_LIMIT_SWITCH;
extern u8 HAS_ALL_REACHED_FLAG;
extern u8 HAS_ALL_DOWN_FLAG;

void Limit_Switch_init();
void initialization_process_full_init();
void initialization_process_front_init();
void initialization_process_back_init();
void DancingMode(int32_t*, int32_t*);
//void INIT_time_protection(u8 dir);
u8 TP_reach_lower_detection();

#endif