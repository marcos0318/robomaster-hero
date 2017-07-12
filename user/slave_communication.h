#ifndef SLAVE_COMMUNICATION_H
#define SLAVE_COMMUNICATION_H
#include "data_monitor.h"
#include "Dbus.h"
#include "stdbool.h"
#include "hero_param.h"
#include "gimbal_control.h"

extern volatile u32 G_counter;
extern volatile u8 state_delay;

extern volatile u8 upper_pneumatic_state ;
extern volatile u8 lower_pneumatic_state ;
extern u8 lower_pneumatic_prev ;
extern u8 upper_pneumatic_prev ;



//INTO_RI_MODE: lower pneumatic timer and flag
extern volatile u8 INTO_RI_LPneu_flag;
extern volatile u32 INTO_RI_LPneu_timer;

//SPEED_LIMITATION: lower pneumatic timer and flag
extern volatile u8 SPEED_LIMITATION_LPneu_flag;
extern volatile u32 SPEED_LIMITATION_LPneu_timer;

//VERTICAL_PNEUMATIC_WITHDRAWS: upper horinzontal peumatic timer and flag
extern volatile u8 VERTICAL_PNEUMATIC_WITHDRAWS_UHPneu_LM_flag;
extern volatile u32 VERTICAL_PNEUMATIC_WITHDRAWS_UHPneu_LM_timer;

//for flash memory usage
extern volatile int16_t LOAD_FLASH;
extern volatile int16_t step;
extern volatile u8 RC_CTRL_SHIFT;
extern volatile u8 RC_CTRL;

extern enum modeControl{
	
	RUNNING_MODE,												//0
	INTO_RI_MODE,												//1
	BACK_WHEEL_UP,											//2
	FRONT_WHEEL_UP,											//3
	SPEED_LIMITATION,										//4
	UPPER_HORIZONTAL_PNEUMATIC_EXTENDS,  					//5
	CATCH_GOLF,													//6
	DANCING_MODE,												//7
	VERTICAL_PNEUMATIC_WITHDRAWS,								//8
	DOWN_FRONT_WHEEL,										//9
	DOWN_BACK_WHEEL,											//10
	REVERSE_RUNNING_MODE,									//11
	LOWER_PNEUMATIC,											//12
	BACK_WHEEL_DOWN,										//13
	FRONT_WHEEL_DOWN										//14
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
//	e. gyro open-looped
//	f. lower pneumatic extended
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
#endif
