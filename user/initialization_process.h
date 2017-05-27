#include "gpio.h"


#define Left_front_LM &PC2
#define Left_back_LM &PC3
#define Right_front_LM &PB15
#define Right_back_LM &PB14


void Limit_Switch_init();
void initialization_process_full_init();
void initialization_process_front_init();
void initialization_process_back_init();