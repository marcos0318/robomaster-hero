#ifndef GOONSTAGE_H
#define GOONSTAGE_H
#include "Lifting_Motor_Control.h"
#include "initialization_process.h"
#include "ticks.h"
extern volatile uint8_t INIT_FLAG ;		//to be modified by initialization_process
void readFeedback();
void speedProcess();
void setSetpoint();
uint8_t num_of_touch(const GPIO*);

#endif