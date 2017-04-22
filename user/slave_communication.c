#ifndef SLAVE_COMMUNICATION_H
#define SLAVE_COMMUNICATION_H
#include "slave_communication.h"
int16_t checkSetpoint(int16_t a, bool dir){

	if(dir) 
		if(a > LiftingMotorSetpointLimit - 200) a = LiftingMotorSetpointLimit - 1;
		else 																		a += 200;
	
	else 
		if(a < 200) 														a = 0;
		else 																		a -= 200;
	
	return a;
}


void transmit(){
		if (DBUS_CheckPush(KEY_SHIFT)) { //SHIFT is pressed

			if(DBUS_CheckPush(KEY_R)){
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
				DataMonitor_Send(5, 0);
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
			else if(!lower_pneumatic_prev && DBUS_CheckPush(KEY_F)){
				//pneumatic
				lower_pneumatic_state = !lower_pneumatic_state;
				pneumatic_control(1, lower_pneumatic_state);
				pneumatic_control(2, lower_pneumatic_state);
			}
			else if(!upper_pneumatic_prev && DBUS_CheckPush(KEY_E)){
				//pneumatic
				if(upper_pneumatic_state == 0){
					DataMonitor_Send(16, 0);
					upper_pneumatic_state = 1;
					pneumatic_control(3, true);
				}
				else if(upper_pneumatic_state == 1){
					DataMonitor_Send(17, 0);
					upper_pneumatic_state = 2;
				}
				else if(upper_pneumatic_state == 2){
					DataMonitor_Send(18,0);
					upper_pneumatic_state = 0;
					pneumatic_control(3, false);
				}
			}
		
		}

		else { //SHIFT is not pressed
			if(DBUS_CheckPush(KEY_R)){
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = UP_SETPOINT/8;
				DataMonitor_Send(0xFF, LiftingMotorSetpoint[0]);        //GO_ON_STAGE_ONE_KEY
			}
			else if(DBUS_CheckPush(KEY_F)){
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = UP_SETPOINT/8;
				LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = MID_SETPOINT/8;
				DataMonitor_Send(0xFE, LiftingMotorSetpoint[0]);        //GO_DOWN_STAGE_ONE_KEY
			}
			//else if(DBUS_CheckPush(KEY_A)){
				//DataMonitor_Send(0xFD,LiftingMotorSetpoint[0]);   //BREAK
			//}
			else if(DBUS_CheckPush(KEY_X)){
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
			
			lower_pneumatic_prev = DBUS_CheckPush(KEY_SHIFT) && DBUS_CheckPush(KEY_F);
			upper_pneumatic_prev = DBUS_CheckPush(KEY_SHIFT) && DBUS_CheckPush(KEY_E);

			
}



#endif