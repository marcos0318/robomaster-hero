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
	
	RUNNING_MODE,												//0
	INTO_RI_MODE,												//1
	ON_RI_MODE,													//2
	BACK_WHEEL_UP,											//3
	FRONT_WHEEL_UP,											//4
	SPEED_LIMITATION,										//5
	PRE_CATCH_GOLF,											//6
	CATCH_GOLF,													//7
	DANCING_MODE,												//8
	VERTICAL_PNEUMATIC_WITHDRAWS,				//9
	LOADED,															//10
	LIFTING_MOTOR_DOWN,									//11
	DOWN_FRONT_WHEEL,										//12
	DOWN_BACK_WHEEL											//13
} HERO;
//0. RUNNING_MODE
//  a. withdraw lower pneumatic
//  b. LiftingMotor all go down
//  c. filter_rate_limit = speed_multiplier = FOR_JPOHN_MAX_RUNNING_SPEED 
//  d. SHIFT_F may directly go to this step
//  e. follow gyro
//1. INTO_RI_MODE (prepare to go into the resource island mode)
//	a. in front of timber pile
//	b. four LiftingMotors go up
//	c. camera towards timber pile
//	d. QWEASD reversed
//2. ON_RI_MODE (prepare to go on the resource island mode continued)
//	a.gyro open-looped
//	b. pneumatic extended
//3. BACK_WHEEL_UP (back wheels go on the stage)
//	a. back wheels LiftingMotors go up
//4. FRONT_WHEEL_UP (back wheels and pneumatic have already gone up)
//	a. front wheels LiftingMotors go up
//5. SPEED_LIMITATION (all wheels on stage)
//	a. set speed limit for the wheels
//  b. all lifting motors go up
//  c. SHIFT+G may directly go to this stage
//6. PRE_CATCH_GOLF (ready to inhale (WTF...) Golf)
//  a. withdraw lower pneumatics
//	b. upper pneumatic still not extended
//	c. readjust camera position
//	d. friction wheel off
//7. CATCH_GOLF (inhale Golf mode)
//	a. upper front back pneumatic  extends
//  b. lower pneumatic withdrew
//	c. friction wheel on
//  d. vertical pneumatic extends
//  e. set speed limit, and also ned to reverse moving direction
//8. DANCING_MODE
//  a. dancing mode begins
//9. VERTICAL_PNEUMATIC_WITHDRAWS
//  a. vertical pneumatic withdraws
//  b. dancing mode ends
//10. LOADED (already loaded)
//	a. friction wheel off	
//	b. withdraw upper front back pneumatic
//	c. extend lower pneumatic
//  d. chasis without gyro, unfollow gimbal
//  e. speed limitation, but QWEASD is not reversed
//11. LIFTING_MOTOR_DOWN (all Lifting Motors go down)
//10. DOWN_FRONT_WHEEL
//11. DOWN_BACK_WHEEL


	
void transmit();
void state_control();
void switch_and_send();