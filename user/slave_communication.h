#include "data_monitor.h"
#include "Dbus.h"
#include "stdbool.h"
#include "hero_param.h"

extern volatile uint8_t upper_pneumatic_state ;
extern volatile bool lower_pneumatic_state ;
extern bool lower_pneumatic_prev ;
extern bool upper_pneumatic_prev ;
void transmit();
void pneumatic_state_init();