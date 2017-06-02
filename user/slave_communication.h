#include "data_monitor.h"
#include "Dbus.h"
#include "stdbool.h"
#include "hero_param.h"

extern volatile uint8_t upper_pneumatic_state ;
extern volatile bool lower_pneumatic_state ;
extern bool lower_pneumatic_prev ;
extern bool upper_pneumatic_prev ;
//extern enum modeControl HERO;
extern enum modeControl{
	
	RUNNING_MODE,
	INTO_RI_MODE,
	ON_RI_MODE,
	BACK_WHEEL_UP,
	FRONT_WHEEL_UP,
	SPEED_LIMITATION,
	PRE_CATCH_GOLF,
	CATCH_GOLF,
	VERTICAL_PNEUMATIC_EXTENDS,
	VERTICAL_PNEUMATIC_WITHDRAWS,
	LOADED,
	LIFTING_MOTOR_DOWN,
	DOWN_FRONT_WHEEL,
	DOWN_BACK_WHEEL
} HERO;
//1. prepare to go into the resource island mode
//	a. in front of timber pile
//	b. four LiftingMotors go up
//	c. camera towards timber pile
//	d. press control in order to reverse QWEASD
//2. prepare to go on the resource island mode
//	a.gyro open-looped
//	b. pneumatic extended
//3. back wheels go on the stage
//	a. back wheels LiftingMotors go up
//4. back wheels and pneumatic have already gone up
//	a. front wheels LiftingMotors go up
//5. all wheels on stage
//	a. set speed limit for the wheels
//  b. all lifting motors go up
//6. ready to inhale (WTF...) Golf
//	a. extend gripper pneumatic
//	b. readjust camera position
//	c. friction wheel off
//7. inhale Golf mode
//	a. add one more freedom, may need to extend this pneumatic
//	b. friction wheel on
//8. already loaded
//	a. friction wheel off	
//	b. withdraw upper pneumatic
//	c. extend lower pneumatic
//9. all Lifting Motors go down
//10.
//11.


	
void transmit();
void state_control();
void switch_and_send();