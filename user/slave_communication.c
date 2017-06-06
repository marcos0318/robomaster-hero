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

//INTO_RI_MODE: lower pneumatic timer and flag
volatile u8 INTO_RI_LPneu_flag = 0;
volatile u32 INTO_RI_LPneu_timer = 0;

//SPEED_LIMITATION: lower pneumatic timer and flag
volatile u8 SPEED_LIMITATION_LPneu_flag = 0;
volatile u32 SPEED_LIMITATION_LPneu_timer = 0;

//VERTICAL_PNEUMATIC_WITHDRAWS: upeer horinzontal peumatic timer and flag
volatile u8 VERTICAL_PNEUMATIC_WITHDRAWS_UHPneu_LM_flag = 0;
volatile u32 VERTICAL_PNEUMATIC_WITHDRAWS_UHPneu_LM_timer = 0;


u8 G_counter_for_John=0;


enum modeControl HERO = RUNNING_MODE;


int16_t checkSetpoint(int16_t a, bool dir){

	if(dir) 
		if(a > LiftingMotorSetpointLimit - 200) a = LiftingMotorSetpointLimit - 1;
		else 																		a += 200;
	
	else 
		if(a < 200) 														a = 0;
		else 																		a -= 200;
	
	return a;
}

void switch_and_send()
{
	switch(HERO){
		case RUNNING_MODE:
			ChasisFlag=1;
			GimbalFlag = 3;
			filter_rate_limit = FOR_JOHN_MAX_RUNNING_SPEED;
			speed_multiplier = FOR_JOHN_MAX_RUNNING_SPEED;
			//RC_dir_multiplier = 1;
			//withdraw lower pneumatic
			lower_pneumatic_state=false;
			pneumatic_control(1, 0);
			pneumatic_control(2, 0);
			pneumatic_control(3, 0);
			pneumatic_control(4, 0);
			LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
			DataMonitor_Send(5, 0);
			//speed limit in chasis control
			break;
		case INTO_RI_MODE:
			//LiftingMotors go up
			LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = UP_SETPOINT/8;
			DataMonitor_Send(0xFF, LiftingMotorSetpoint[0]);        //GO_ON_STAGE_ONE_KEY
			//reverse QWEASD
			filter_rate_limit = FOR_JOHN_INTO_RI_MAX_SPEED;
			speed_multiplier = -FOR_JOHN_INTO_RI_MAX_SPEED;
			//turn off gyro
			ChasisFlag=3;
			//extend lower pneumatic
			//do it in the interrupt
			INTO_RI_LPneu_flag = 1;
			INTO_RI_LPneu_timer = TIM_7_Counter;
			
			break;
		case BACK_WHEEL_UP:
			LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = DOWN_SETPOINT/8;
			DataMonitor_Send(0xFB, LiftingMotorSetpoint[2]);		//ONE_KEY_DOWN_BACK			
			break;
		case FRONT_WHEEL_UP:
			LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = DOWN_SETPOINT/8;
			DataMonitor_Send(0xFC, LiftingMotorSetpoint[0]);		//ONE_KEY_DOWN_FRONT						
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
			upper_pneumatic_state = 0;
			pneumatic_control(3, true);	
			pneumatic_control(4, false);
			//withdraw lower pneumatic
			//do it in the interrupt
			SPEED_LIMITATION_LPneu_flag = 1;
			SPEED_LIMITATION_LPneu_timer = TIM_7_Counter;			
			break;
		case CATCH_GOLF:
			DataMonitor_Send(28,0);			//turn on friciton wheel
			//upper_pneumatic_state = 0;
			//pneumatic_control(3, true);		
			pneumatic_control(4, true);
			lower_pneumatic_state=false;
			pneumatic_control(1, 0);
			pneumatic_control(2, 0);
			filter_rate_limit = FOR_JOHN_INTO_RI_MAX_SPEED;
			speed_multiplier = -FOR_JOHN_INTO_RI_MAX_SPEED;
			break;
		case DANCING_MODE:
			pneumatic_control(4, true);
			DataMonitor_Send(63, 0);
			//LiftingMotors oscillate
			ChasisFlag = 3;
			filter_rate_limit = FOR_JOHN_INTO_RI_MAX_SPEED;
			speed_multiplier = -FOR_JOHN_INTO_RI_MAX_SPEED;
			break;
		case VERTICAL_PNEUMATIC_WITHDRAWS:
			pneumatic_control(4, false);
			DataMonitor_Send(64, 0);
			//LiftingMotors stop oscillate
			//and turn off the friction wheel			
			//DataMonitor_Send(26, 0);	// turn off friction wheel
			VERTICAL_PNEUMATIC_WITHDRAWS_UHPneu_LM_flag = 1;
			VERTICAL_PNEUMATIC_WITHDRAWS_UHPneu_LM_timer = TIM_7_Counter;
			lower_pneumatic_state=true;
			pneumatic_control(1, 1);
			pneumatic_control(2, 1);
			ChasisFlag=4;	
			filter_rate_limit = FOR_JOHN_INTO_RI_MAX_SPEED;
			speed_multiplier = FOR_JOHN_INTO_RI_MAX_SPEED;
			
			break;
		case DOWN_FRONT_WHEEL:
			LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = UP_SETPOINT/8;
			DataMonitor_Send(0xFA, LiftingMotorSetpoint[0]);		//ONE_KEY_UP_FRONT		
			break;
		case DOWN_BACK_WHEEL:
			LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = UP_SETPOINT/8;
			DataMonitor_Send(0xF9, LiftingMotorSetpoint[2]);		//ONE_KEY_UP_BACK		
			break;
		
	}
}

void state_control(){
	if(DBUS_CheckPush(KEY_CTRL) && (DBUS_CheckPush(KEY_F)||DBUS_CheckPush(KEY_G)||DBUS_CheckPush(KEY_C)||DBUS_CheckPush(KEY_V)||DBUS_CheckPush(KEY_SHIFT))){
		KEY_G_PREV=DBUS_CheckPush(KEY_G);
		KEY_F_PREV=DBUS_CheckPush(KEY_F);
		KEY_SHIFT_F_PREV = DBUS_CheckPush(KEY_F) && DBUS_CheckPush(KEY_SHIFT);
		KEY_SHIFT_G_PREV = DBUS_CheckPush(KEY_G) && DBUS_CheckPush(KEY_SHIFT);
		transmit();
		return;
	}
	if(DBUS_CheckPush(KEY_G)) G_counter_for_John+=1;
	if(KEY_G_PREV && !DBUS_CheckPush(KEY_G)) {
		if((G_counter_for_John > 30) && HERO == RUNNING_MODE) {
			HERO=INTO_RI_MODE;
			state_switch=true;
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
	if(!DBUS_CheckPush(KEY_SHIFT) && DBUS_CheckPush(KEY_G)&&(!KEY_G_PREV)){
			if(HERO==RUNNING_MODE){
			}
			else 
			{
				if(HERO!=DOWN_BACK_WHEEL)
					HERO+=1;
				else HERO=RUNNING_MODE;
			}
	}
	if(!DBUS_CheckPush(KEY_SHIFT) && DBUS_CheckPush(KEY_F)&&(!KEY_F_PREV)){
			if(HERO!=RUNNING_MODE)
				HERO-=1;	
	}
	if(DBUS_CheckPush(KEY_F) && DBUS_CheckPush(KEY_SHIFT) && (!KEY_SHIFT_F_PREV)){
		  HERO=RUNNING_MODE;  		
			SHIFT_F=true;
	}
	else SHIFT_F=false;
	if(DBUS_CheckPush(KEY_G) && DBUS_CheckPush(KEY_SHIFT) && (!KEY_SHIFT_G_PREV)){

			HERO=SPEED_LIMITATION;
			SHIFT_G=true;
	}
	else SHIFT_G=false;
	
	//What's the condition to execute this switch???
	//may be press G or press the key to switch back state, and of course, prev is needed
	if((DBUS_CheckPush(KEY_G)&&!KEY_G_PREV) || (DBUS_CheckPush(KEY_F)&&!KEY_F_PREV) || SHIFT_F || SHIFT_G || state_switch){
			switch_and_send();
	}	
	
	
	KEY_G_PREV=DBUS_CheckPush(KEY_G);
	KEY_F_PREV=DBUS_CheckPush(KEY_F);
	KEY_SHIFT_F_PREV = DBUS_CheckPush(KEY_F) && DBUS_CheckPush(KEY_SHIFT);
	KEY_SHIFT_G_PREV = DBUS_CheckPush(KEY_G) && DBUS_CheckPush(KEY_SHIFT);

}

void transmit(){
		if (DBUS_CheckPush(KEY_CTRL) && !DBUS_CheckPush(KEY_SHIFT) && (DBUS_CheckPush(KEY_F)||DBUS_CheckPush(KEY_G)||DBUS_CheckPush(KEY_C)||DBUS_CheckPush(KEY_V))) 
		{
			int16_t key_bit = 0;
			if(DBUS_CheckPush(KEY_F) ){
				LiftingMotorSetpoint[0] = checkSetpoint(LiftingMotorSetpoint[0], false);
				key_bit |= 1<<0; 
			}
			if(DBUS_CheckPush(KEY_G) ){
				LiftingMotorSetpoint[1] = checkSetpoint(LiftingMotorSetpoint[1], false);
				key_bit |= 1<<1;
			} 
			if(DBUS_CheckPush(KEY_C) ){
				LiftingMotorSetpoint[3] = checkSetpoint(LiftingMotorSetpoint[3], false);
				key_bit |= 1<<2;
			}
			if(DBUS_CheckPush(KEY_V) ){
				LiftingMotorSetpoint[2] = checkSetpoint(LiftingMotorSetpoint[2], false);
				key_bit |= 1<<3;				
			}
			DataMonitor_Send(19, key_bit);
		}
		else
		if (DBUS_CheckPush(KEY_SHIFT)) { //SHIFT is pressed
			if(DBUS_CheckPush(KEY_R) && DBUS_CheckPush(KEY_CTRL)){
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
				DataMonitor_Send(90, 0);
			}
			else if(DBUS_CheckPush(KEY_R) && !DBUS_CheckPush(KEY_CTRL)){
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
				DataMonitor_Send(5, 0);
			}
			else if(DBUS_CheckPush(KEY_CTRL) && (DBUS_CheckPush(KEY_F)||DBUS_CheckPush(KEY_G)||DBUS_CheckPush(KEY_C)||DBUS_CheckPush(KEY_V))){
				int16_t key_bit = 0;
				if(DBUS_CheckPush(KEY_F)){
					LiftingMotorSetpoint[0] = checkSetpoint(LiftingMotorSetpoint[0], true);
					key_bit |= 1<<0;
				}
				if(DBUS_CheckPush(KEY_G)){
					LiftingMotorSetpoint[1] = checkSetpoint(LiftingMotorSetpoint[1], true);
					key_bit |= 1<<1;
				} 
				if(DBUS_CheckPush(KEY_C)){
					LiftingMotorSetpoint[3] = checkSetpoint(LiftingMotorSetpoint[3], true);
					key_bit |= 1<<2;
				}
				if(DBUS_CheckPush(KEY_V)){
					LiftingMotorSetpoint[2] = checkSetpoint(LiftingMotorSetpoint[2], true);
					key_bit |= 1<<3;
				}
				DataMonitor_Send(0x14, key_bit);
			}
			else if(DBUS_CheckPush(KEY_Z)){
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = checkSetpoint(LiftingMotorSetpoint[0], true);
				DataMonitor_Send(0, LiftingMotorSetpoint[0]);
			}
			else if(DBUS_CheckPush(KEY_X)){
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = checkSetpoint(LiftingMotorSetpoint[0], false);
				DataMonitor_Send(0, LiftingMotorSetpoint[0]);
			}
			else if(DBUS_CheckPush(KEY_C)){
				LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = checkSetpoint(LiftingMotorSetpoint[2], true);
				DataMonitor_Send(2, LiftingMotorSetpoint[2]);
			}
			else if(DBUS_CheckPush(KEY_V)){
				LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = checkSetpoint(LiftingMotorSetpoint[2], false);
				DataMonitor_Send(2, LiftingMotorSetpoint[2]);
			}
			else if(!lower_pneumatic_prev && DBUS_CheckPush(KEY_Q)){
				//pneumatic
				lower_pneumatic_state = !lower_pneumatic_state;
				pneumatic_control(1, lower_pneumatic_state);
				pneumatic_control(2, lower_pneumatic_state);
			}
			else if(!upper_pneumatic_prev && DBUS_CheckPush(KEY_E)){
				//pneumatic
				if(upper_pneumatic_state == 0){
					DataMonitor_Send(26, 0);	// turn off friction wheel
					upper_pneumatic_state = 1;
					pneumatic_control(3, true);	//extend the pneumatic
				}
				else if(upper_pneumatic_state == 1){
					DataMonitor_Send(27, 0);	//friction wheel still off
					upper_pneumatic_state = 2;	//pneumatic still extended
				}
				else if(upper_pneumatic_state == 2){
					DataMonitor_Send(28,0);		//turn on friction wheel
					upper_pneumatic_state = 0;
					pneumatic_control(3, false);	//withdraw pneumatic
				}
				else
					DataMonitor_Send(0x55, 0);	//keep communication
			}
		
		}

		else { //SHIFT is not pressed
			/*
			if(DBUS_CheckPush(KEY_R)){
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = UP_SETPOINT/8;
				DataMonitor_Send(0xFF, LiftingMotorSetpoint[0]);        //GO_ON_STAGE_ONE_KEY
			}
			else
			*/
			/*			
			if(DBUS_CheckPush(KEY_E)){
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = UP_SETPOINT/8;
				LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = MID_SETPOINT/8;
				DataMonitor_Send(0xFE, LiftingMotorSetpoint[0]);        //GO_DOWN_STAGE_ONE_KEY
			}
			*/
			//else if(DBUS_CheckPush(KEY_A)){
				//DataMonitor_Send(0xFD,LiftingMotorSetpoint[0]);   //BREAK
			//}
//			else
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
			else 
				DataMonitor_Send(0x55, 0);	//keep communication
			

			
		}

		lower_pneumatic_prev = DBUS_CheckPush(KEY_SHIFT) && DBUS_CheckPush(KEY_Q);
		upper_pneumatic_prev = DBUS_CheckPush(KEY_SHIFT) && DBUS_CheckPush(KEY_E);
}


