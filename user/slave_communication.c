#include "slave_communication.h"
#include "pneumatic.h"
#include "1.8 TFT_display.h"

volatile uint8_t upper_pneumatic_state = 0 ;
volatile u8 lower_pneumatic_state = 0 ;
u8 lower_pneumatic_prev = 0 ;
u8 upper_pneumatic_prev = 0 ;

volatile u8 KEY_G_PREV = 0;
volatile u8 KEY_F_PREV = 0;
volatile u8 KEY_SHIFT_G_PREV = 0;
volatile u8 KEY_SHIFT_F_PREV = 0;
volatile u8 SHIFT_F = 0;
volatile u8 SHIFT_G = 0;


volatile u8 state_switch = 0;
volatile u8 state_delay = 0; 
volatile u32 G_counter = 0;

//for flash memory usage
volatile int16_t LOAD_FLASH = 0;
volatile int16_t step = 0;
volatile u8 RC_CTRL_SHIFT = 0;
volatile u8 RC_CTRL = 0;

//INTO_RI_MODE: lower pneumatic timer and flag
volatile u8 INTO_RI_LPneu_flag = 0;
volatile u32 INTO_RI_LPneu_timer = 0;

//SPEED_LIMITATION: lower pneumatic timer and flag
volatile u8 SPEED_LIMITATION_LPneu_flag = 0;
volatile u32 SPEED_LIMITATION_LPneu_timer = 0;

//VERTICAL_PNEUMATIC_WITHDRAWS: upper horinzontal pneumatic and LiftingMotor timer and flag
volatile u8 VERTICAL_PNEUMATIC_WITHDRAWS_UHPneu_LM_flag = 0;
volatile u32 VERTICAL_PNEUMATIC_WITHDRAWS_UHPneu_LM_timer = 0;


u8 G_counter_for_John=0;

enum modeControl HERO = REVERSE_RUNNING_MODE;



void switch_and_send()
{
	switch(HERO){
		case RUNNING_MODE:		
			ChasisFlag = 1;
			if (DBUS_ReceiveData.mouse.press_right) {
				ChasisFlag = 2;
			}
			GimbalFlag = 3;
		  direction = -output_angle*upperTotal/3600;
			filter_rate_limit = FOR_JOHN_MAX_RUNNING_SPEED;
			speed_multiplier = FOR_JOHN_MAX_RUNNING_SPEED;
			//withdraw lower pneumatic
			lower_pneumatic_state = false;
			pneumatic_control(1, 0);
			pneumatic_control(3, 0);
			pneumatic_control(4, 0);
			LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
			DataMonitor_Send(5, 1);
			//speed limit in chasis control
			break;
		case INTO_RI_MODE:
			//LiftingMotors go up
			LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = UP_SETPOINT/8;
			DataMonitor_Send(0xFF, 0);        //GO_ON_STAGE_ONE_KEY
			//reverse QWEASD
			filter_rate_limit = FOR_JOHN_INTO_RI_MAX_SPEED;
			speed_multiplier = -FOR_JOHN_INTO_RI_MAX_SPEED;
			//turn off gyro
			ChasisFlag = 3;
			//extend lower pneumatic
			//do it in the interrupt
			//INTO_RI_LPneu_flag = 1;
			//INTO_RI_LPneu_timer = TIM_7_Counter;
			lower_pneumatic_state=true;
			pneumatic_control(1, 1);
			break;
		case BACK_WHEEL_UP:
			LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
            LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = UP_SETPOINT/8;
			DataMonitor_Send(70, 1);		//ONE_KEY_DOWN_BACK			
			break;
		case FRONT_WHEEL_UP:
			LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = DOWN_SETPOINT/8;
			if(LOAD_FLASH == 0)	
				DataMonitor_Send(71, 0);		//ONE_KEY_DOWN_FRONT		
			else if(LOAD_FLASH == 1)
				DataMonitor_Send(71, 1);
			LOAD_FLASH = 0;
			break;
		case SPEED_LIMITATION:
			ChasisFlag = 3;
			filter_rate_limit = FOR_JOHN_INTO_RI_MAX_SPEED;
			speed_multiplier = -FOR_JOHN_INTO_RI_MAX_SPEED;
		  //all lifting motor go up 
			//up to the limit switch
			//need to add new number
			LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = UP_DOWN_DISTANCE/8;
			DataMonitor_Send(69, 0);        //up to the limit switch
			//LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = UP_SETPOINT/8;
			//DataMonitor_Send(0xFF, LiftingMotorSetpoint[0]);        //GO_ON_STAGE_ONE_KEY
			
			//withdraw lower pneumatic
			//do it in the interrupt
			SPEED_LIMITATION_LPneu_flag = 1;
			SPEED_LIMITATION_LPneu_timer = TIM_7_Counter;			
			break;
		case UPPER_HORIZONTAL_PNEUMATIC_EXTENDS:
			if(LOAD_FLASH ==1)	
				DataMonitor_Send(72, 2);
			else DataMonitor_Send(72,0);
			LOAD_FLASH = 0;
			upper_pneumatic_state = 0;
			pneumatic_control(3, 1);	
			pneumatic_control(4, 0);
            lower_pneumatic_state=false;
			pneumatic_control(1, 0);
			pneumatic_control(2, 0);

			break;
		case CATCH_GOLF:
			DataMonitor_Send(28,0);			//turn on friciton wheel
			//upper_pneumatic_state = 0;
			//pneumatic_control(3, true);		
			pneumatic_control(4, 1);
			filter_rate_limit = FOR_JOHN_INTO_RI_MAX_SPEED;
			speed_multiplier = -FOR_JOHN_INTO_RI_MAX_SPEED;
			break;
		case DANCING_MODE:
			pneumatic_control(4, 1);
			DataMonitor_Send(63, 0);
			//LiftingMotors oscillate
			ChasisFlag = 3;
			filter_rate_limit = FOR_JOHN_INTO_RI_MAX_SPEED;
			speed_multiplier = -FOR_JOHN_INTO_RI_MAX_SPEED;
			break;
		case VERTICAL_PNEUMATIC_WITHDRAWS:
			pneumatic_control(4, 0);
			DataMonitor_Send(64, 0);
			//LiftingMotors stop oscillate
			//and turn off the friction wheel			
			//DataMonitor_Send(26, 0);	// turn off friction wheel
			VERTICAL_PNEUMATIC_WITHDRAWS_UHPneu_LM_flag = 1;
			VERTICAL_PNEUMATIC_WITHDRAWS_UHPneu_LM_timer = TIM_7_Counter;
			lower_pneumatic_state=true;
			pneumatic_control(1, 0);
			ChasisFlag=4;	
			filter_rate_limit = FOR_JOHN_INTO_RI_MAX_SPEED;
			speed_multiplier = FOR_JOHN_INTO_RI_MAX_SPEED;
			
			break;
		case DOWN_FRONT_WHEEL:
			pneumatic_control(1, 1);
			pneumatic_control(2, 1);
			ChasisFlag=4;	
			LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = UP_SETPOINT/8;
            LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
			filter_rate_limit = FOR_JOHN_INTO_RI_MAX_SPEED;
			speed_multiplier = FOR_JOHN_INTO_RI_MAX_SPEED;
			DataMonitor_Send(70, 0);		//ONE_KEY_UP_FRONT		
			break;
		case DOWN_BACK_WHEEL:
			LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = UP_SETPOINT/8;
			DataMonitor_Send(0xFF, 1);		//ONE_KEY_UP_BACK		
			break;
		case REVERSE_RUNNING_MODE:
			ChasisFlag = 1;
			filter_rate_limit = FOR_JOHN_MAX_RUNNING_SPEED;
			speed_multiplier = -FOR_JOHN_MAX_RUNNING_SPEED;
			GimbalFlag = 3;
			direction = -output_angle*upperTotal/3600;
			lower_pneumatic_state = false;
			pneumatic_control(1, 0);
			pneumatic_control(3, 0);
			pneumatic_control(4, 0);
			DataMonitor_Send(5, 3);
			break;
		case BACK_WHEEL_DOWN:
			pneumatic_control(1, 1);
			pneumatic_control(2, 1);
			LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = UP_SETPOINT/8;
			DataMonitor_Send(0xFB, 0);		//ONE_KEY_UP_BACK		
			break;
		case FRONT_WHEEL_DOWN:
			LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = UP_SETPOINT/8;
            LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
			DataMonitor_Send(0xFF, 0);		//ONE_KEY_UP_FRONT	
			break;
		
	}
}

void state_control(){
	if(state_delay || RC_CTRL || RC_CTRL_SHIFT || (DBUS_CheckPush(KEY_CTRL) && (DBUS_CheckPush(KEY_F)||DBUS_CheckPush(KEY_G)||DBUS_CheckPush(KEY_C)||DBUS_CheckPush(KEY_V)||DBUS_CheckPush(KEY_SHIFT)))){
		KEY_G_PREV=DBUS_CheckPush(KEY_G);
		KEY_F_PREV=DBUS_CheckPush(KEY_F);
		KEY_SHIFT_F_PREV = DBUS_CheckPush(KEY_F) && DBUS_CheckPush(KEY_SHIFT);
		KEY_SHIFT_G_PREV = DBUS_CheckPush(KEY_G) && DBUS_CheckPush(KEY_SHIFT);
		transmit();
		return;
	}
	if(!DBUS_CheckPush(KEY_SHIFT) && DBUS_CheckPush(KEY_G)) G_counter_for_John+=1;
	if(!FOR_JOHN_SHIFT_G_SPECIAL_MODE && !DBUS_CheckPush(KEY_SHIFT) && KEY_G_PREV && !DBUS_CheckPush(KEY_G)) {
		if((G_counter_for_John > 30) && HERO == RUNNING_MODE) {
			HERO=INTO_RI_MODE;
			switch_and_send();
			state_switch=true;
			G_counter = TIM_7_Counter;
			state_delay = 1;
		}
		else state_switch=false;
		G_counter_for_John=0;
	}
	else
	if(!DBUS_CheckPush(KEY_G) && !DBUS_CheckPush(KEY_F)){
		KEY_G_PREV=DBUS_CheckPush(KEY_G);
		KEY_F_PREV=DBUS_CheckPush(KEY_F);
		KEY_SHIFT_F_PREV = DBUS_CheckPush(KEY_F) && DBUS_CheckPush(KEY_SHIFT);
		KEY_SHIFT_G_PREV = DBUS_CheckPush(KEY_G) && DBUS_CheckPush(KEY_SHIFT);
		transmit();
		return;
	}
	if(!FOR_JOHN_SHIFT_G_SPECIAL_MODE && !DBUS_CheckPush(KEY_SHIFT) && DBUS_CheckPush(KEY_G)&&(!KEY_G_PREV) && HERO != RUNNING_MODE){
			
				if(HERO!=DOWN_BACK_WHEEL && HERO != FRONT_WHEEL_DOWN){
					HERO+=1;
					switch_and_send();
				}
				else if(HERO == DOWN_BACK_WHEEL){
					HERO=RUNNING_MODE;
					ChasisFlag = 1;
					GimbalFlag = 3;
					direction = - output_angle*upperTotal/3600;
					filter_rate_limit = FOR_JOHN_MAX_RUNNING_SPEED;
					speed_multiplier = FOR_JOHN_MAX_RUNNING_SPEED;
					//withdraw lower pneumatic
					lower_pneumatic_state = false;
					pneumatic_control(1, 0);
					pneumatic_control(3, 0);
					pneumatic_control(4, 0);
					LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
					DataMonitor_Send(5, 2);
				} else if(HERO == FRONT_WHEEL_DOWN) {
					HERO = INTO_RI_MODE;
					switch_and_send();
				}
				G_counter = TIM_7_Counter;
				state_delay = 1;
			
	}
	if(!DBUS_CheckPush(KEY_SHIFT) && DBUS_CheckPush(KEY_F)&&(!KEY_F_PREV) && HERO != RUNNING_MODE){
    	if(!FOR_JOHN_SHIFT_G_SPECIAL_MODE)    
		{
			if(HERO == RUNNING_MODE || HERO == REVERSE_RUNNING_MODE) {}
			else if(HERO == BACK_WHEEL_DOWN) {
				HERO = REVERSE_RUNNING_MODE;
				pneumatic_control(1, 0);
				DataMonitor_Send(5, 3);
			}
			else if(HERO == FRONT_WHEEL_DOWN) {
				HERO = BACK_WHEEL_DOWN;
				switch_and_send();
			}
			else if(HERO != VERTICAL_PNEUMATIC_WITHDRAWS)
				backState[HERO--]();
			else {
				backState[HERO]();
				HERO = UPPER_HORIZONTAL_PNEUMATIC_EXTENDS;

			}
		}
		else {
			HERO = RUNNING_MODE;
			backState[1]();
		}
	}
	
	KEY_G_PREV=DBUS_CheckPush(KEY_G);
	KEY_F_PREV=DBUS_CheckPush(KEY_F);
	KEY_SHIFT_F_PREV = DBUS_CheckPush(KEY_F) && DBUS_CheckPush(KEY_SHIFT);
	KEY_SHIFT_G_PREV = DBUS_CheckPush(KEY_G) && DBUS_CheckPush(KEY_SHIFT);

}

void transmit(){
		if (RC_CTRL || (DBUS_CheckPush(KEY_CTRL) && !DBUS_CheckPush(KEY_SHIFT) && (DBUS_CheckPush(KEY_F)||DBUS_CheckPush(KEY_G)||DBUS_CheckPush(KEY_C)||DBUS_CheckPush(KEY_V)))) 
		{
			//CTRL + FGCV
			if(step == 0 && !RC_CTRL){
			int16_t key_bit = 0;
			if(DBUS_CheckPush(KEY_F) ){
				LiftingMotorSetpoint[0] -= 200;
				key_bit |= 1<<0; 
			}
			if(DBUS_CheckPush(KEY_G) ){
				LiftingMotorSetpoint[1] -= 200;
				key_bit |= 1<<1;
			} 
			if(DBUS_CheckPush(KEY_C) ){
				LiftingMotorSetpoint[3] -= 200;
				key_bit |= 1<<2;
			}
			if(DBUS_CheckPush(KEY_V) ){
				LiftingMotorSetpoint[2] -= 200;
				key_bit |= 1<<3;				
			}
			DataMonitor_Send(19, key_bit);
			}
			else if(step > 30) 
				DataMonitor_Send(19, step);
		}
		else
		if (RC_CTRL_SHIFT || DBUS_CheckPush(KEY_SHIFT)) { //SHIFT is pressed
			if(RC_CTRL_SHIFT || (DBUS_CheckPush(KEY_CTRL) && (DBUS_CheckPush(KEY_F)||DBUS_CheckPush(KEY_G)||DBUS_CheckPush(KEY_C)||DBUS_CheckPush(KEY_V)))){
				//CTRL + SHIFT + FGCV
				if(step == 0 && !RC_CTRL_SHIFT){
					int16_t key_bit = 0;
					if(DBUS_CheckPush(KEY_F)){
						LiftingMotorSetpoint[0] += 200;
						key_bit |= 1<<0;
					}
					if(DBUS_CheckPush(KEY_G)){
						LiftingMotorSetpoint[1] += 200;
						key_bit |= 1<<1;
					} 
					if(DBUS_CheckPush(KEY_C)){
						LiftingMotorSetpoint[3] += 200;
						key_bit |= 1<<2;
					}
					if(DBUS_CheckPush(KEY_V)){
						LiftingMotorSetpoint[2] += 200;
						key_bit |= 1<<3;
					}
					DataMonitor_Send(0x14, key_bit);
				}
				else if(step >30)
					DataMonitor_Send(0x14, step);
			} 
			else //shift is pressed
			if(DBUS_CheckPush(KEY_R) && DBUS_CheckPush(KEY_CTRL)){
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
				DataMonitor_Send(90, 0);
				//init
			}
			else if(DBUS_CheckPush(KEY_R) && !DBUS_CheckPush(KEY_CTRL)){
				HERO = RUNNING_MODE;
				ChasisFlag = 1;
				if (DBUS_ReceiveData.mouse.press_right) {
					ChasisFlag = 2;
				}
				GimbalFlag = 3;
				direction = -output_angle*upperTotal/3600;
				filter_rate_limit = FOR_JOHN_MAX_RUNNING_SPEED;
				speed_multiplier = FOR_JOHN_MAX_RUNNING_SPEED;
				//withdraw lower pneumatic
				lower_pneumatic_state = false;
				pneumatic_control(1, 0);
				pneumatic_control(3, 0);
				pneumatic_control(4, 0);
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
				DataMonitor_Send(5, 0);
				//all goes to zero
			}
			else DataMonitor_Send(0x55, 0);	//keep communication			
		}

		else { //SHIFT is not pressed
			if(DBUS_CheckPush(KEY_X)){
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = DOWN_SETPOINT/8;
				DataMonitor_Send(0xFC, LiftingMotorSetpoint[0]);		//ONE_KEY_DOWN_FRONT					
			}
			else if(DBUS_CheckPush(KEY_V)){
				LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = DOWN_SETPOINT/8;
				DataMonitor_Send(0xFB, LiftingMotorSetpoint[2]);		//ONE_KEY_DOWN_BACK					
			}
			else if(DBUS_CheckPush(KEY_Z)){
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = UP_SETPOINT/8;
				DataMonitor_Send(0xFA, LiftingMotorSetpoint[0]);		//ONE_KEY_UP_FRONT						
			}
			else if(DBUS_CheckPush(KEY_C)){
				LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = UP_SETPOINT/8;
				DataMonitor_Send(0xF9, LiftingMotorSetpoint[2]);		//ONE_KEY_UP_BACK					
			}	
			else DataMonitor_Send(0x55, 0);	//keep communication	
		}

		
}


