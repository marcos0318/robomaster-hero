#include "data_monitor.h"
#include "Dbus.h"

extern volatile uint8_t upper_pneumatic_state = 0;
extern volatile bool lower_pneumatic_state = false;
extern bool lower_pneumatic_prev = false;
extern bool upper_pneumatic_prev = false;
void transmit(){};