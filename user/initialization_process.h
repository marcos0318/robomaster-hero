#include "gpio.h"


#define LeftFront &PC2
#define LeftBack &PC3
#define RightFront &PB15
#define RightBack &PB14

extern uint8_t LeftFrontExpand ;
extern uint8_t RightFrontExpand ;
extern uint8_t RightBackExpand ;
extern uint8_t LeftBackExpand ;


void Limit_Switch_init();
void initialization_process_full_init();
void initialization_process_front_init();
void initialization_process_back_init();
