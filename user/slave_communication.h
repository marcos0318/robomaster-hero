#include "data_monitor.h"
#include "Dbus.h"
#include "stdbool.h"
#include "hero_param.h"

extern volatile uint8_t upper_pneumatic_state ;
extern volatile bool lower_pneumatic_state ;
extern bool lower_pneumatic_prev ;
extern bool upper_pneumatic_prev ;


//1. prepare to go into the resource island mode
//	a. in front of timber pile
//	b. four LiftingMotors go up
//	c. pneumatic extended
//	d. camera towards timber pile
//	e. press control in order to reverse QWEASD
//2. prepare to go on the resource island mode
//	gyro open-looped
//3. back wheels go on the stage
//	a. back wheels LiftingMotors go up
//4. back wheels and pneumatic have already gone up
//	a. front wheels LiftingMotors go up
//5. all wheels on stage
//	a. set speed limit for the wheels
//6. ready to inhale (WTF...) Golf
//	a. extend gripper pneumatic
//	b. readjust camera position
//	c. friction wheel off
//7. inhale Golf mode
//	a. add one more freedom, may need to extend this pneumatic
//	b. friction wheel on
//8. already loaded
//	a. friction wheel off	
//	b. withdraw pneumatic


	
void transmit();
void state_control();