#include "initialization_process.h"
#include "GoOnStage.h"
uint8_t LeftFrontExpand = 1;
uint8_t RightFrontExpand = 1;
uint8_t RightBackExpand = 1;
uint8_t LeftBackExpand = 1;
uint8_t LeftFrontReach = 0;
uint8_t RightFrontReach = 0;
uint8_t RightBackReach = 0;
uint8_t LeftBackReach = 0;

uint8_t LeftFrontReachLower = 0;
uint8_t RightFrontReachLower = 0;
uint8_t RightBackReachLower = 0;
uint8_t LeftBackReachLower = 0;

uint8_t LeftFrontReachUpper = 1;
uint8_t RightFrontReachUpper = 1;
uint8_t RightBackReachUpper = 1;
uint8_t LeftBackReachUpper = 1;

u8 ALL_TO_LIMIT_SWITCH = 0;

void Limit_Switch_init(){
	
	BUTTON_init(LeftFront);
	BUTTON_init(LeftBack);
	BUTTON_init(RightFront);
	BUTTON_init(RightBack);

}

void LF_init(){
	if(LeftFrontExpand){
		if(num_of_touch(LeftFront)>15){
			LeftFrontExpand = 0;
			LeftFrontReach = 1;
			LiftingMotorPositionLimit[0] = CM1Encoder.ecd_angle;
			LiftingMotorBias[0] = LiftingMotorPositionLimit[0] - UP_DOWN_DISTANCE;
			LiftingMotorUpperLimit[0] = LiftingMotorBias[0] + UP_SETPOINT;
			//LiftingMotorPositionSetpoint[0] = LiftingMotorBias[0] + TOTALLY_DOWN_SETPOINT;
			PIDClearError(&LiftingMotorState[0]);
			fpidClearError(&LiftingMotorPositionState[0]);
			LiftingMotorSpeedSetpointBuffered[0] = 0;
			LiftingMotorSpeedSetpoint[0] = 0;
			LiftingMotorOutput[0] = 0;
			LiftingMotorPositionSetpoint[0] = CM1Encoder.ecd_angle - DOWN_SETPOINT;
		}
		else {
			LiftingMotorPositionSetpoint[0] = LiftingMotorBias[0] + RASING_HEIGHT_FOR_INITIALIZATION;
		}
	}
	//if(LeftFrontReach){}

}
void RF_init(){
	if(RightFrontExpand){
		if(num_of_touch(RightFront)>15){
			RightFrontExpand = 0;
			RightFrontReach = 1;
			LiftingMotorPositionLimit[1] = CM2Encoder.ecd_angle;
			LiftingMotorBias[1] = LiftingMotorPositionLimit[1]  - UP_DOWN_DISTANCE;
			LiftingMotorUpperLimit[1] = LiftingMotorBias[1] + UP_SETPOINT;
			//LiftingMotorPositionSetpoint[1] = LiftingMotorBias[1] + TOTALLY_DOWN_SETPOINT;
			PIDClearError(&LiftingMotorState[1]);
			fpidClearError(&LiftingMotorPositionState[1]);
			LiftingMotorSpeedSetpointBuffered[1] = 0;
			LiftingMotorSpeedSetpoint[1] = 0;
			LiftingMotorOutput[1] = 0;
			LiftingMotorPositionSetpoint[1] = CM2Encoder.ecd_angle - DOWN_SETPOINT;
			
		}
		else {
			LiftingMotorPositionSetpoint[1] = LiftingMotorBias[1] + RASING_HEIGHT_FOR_INITIALIZATION;
		}
	}


}

void LB_init(){
	if(LeftBackExpand){
		if(num_of_touch(LeftBack)>15){
			LeftBackExpand = 0;
			LeftBackReach = 1;
			LiftingMotorPositionLimit[3] = CM4Encoder.ecd_angle;
			LiftingMotorBias[3] = LiftingMotorPositionLimit[3] - UP_DOWN_DISTANCE;
			LiftingMotorUpperLimit[3] = LiftingMotorBias[3] + UP_SETPOINT;
			//LiftingMotorPositionSetpoint[3] = LiftingMotorBias[3] + TOTALLY_DOWN_SETPOINT;
			PIDClearError(&LiftingMotorState[3]);
			fpidClearError(&LiftingMotorPositionState[3]);
			LiftingMotorSpeedSetpointBuffered[3] = 0;
			LiftingMotorSpeedSetpoint[3] = 0;
			LiftingMotorOutput[3] = 0;
			LiftingMotorPositionSetpoint[3] = CM4Encoder.ecd_angle - DOWN_SETPOINT;
			
		}
		else {
			LiftingMotorPositionSetpoint[3] = LiftingMotorBias[3] + RASING_HEIGHT_FOR_INITIALIZATION;
		}
	}
	
}

void RB_init(){
	if(RightBackExpand){
		if(num_of_touch(RightBack)>15){
			RightBackExpand = 0;
			RightBackReach = 1;
			LiftingMotorPositionLimit[2] = CM3Encoder.ecd_angle;
			LiftingMotorBias[2] = LiftingMotorPositionLimit[2]  - UP_DOWN_DISTANCE;
			LiftingMotorUpperLimit[2] = LiftingMotorBias[2] + UP_SETPOINT;
			//LiftingMotorPositionSetpoint[2] = LiftingMotorBias[2] + TOTALLY_DOWN_SETPOINT;
			PIDClearError(&LiftingMotorState[2]);
			fpidClearError(&LiftingMotorPositionState[2]);
			LiftingMotorSpeedSetpointBuffered[2] = 0;
			LiftingMotorSpeedSetpoint[2] = 0;
			LiftingMotorOutput[2] = 0;
			LiftingMotorPositionSetpoint[2] = CM3Encoder.ecd_angle - DOWN_SETPOINT;
		}
		else {
			LiftingMotorPositionSetpoint[2] = LiftingMotorBias[2] + RASING_HEIGHT_FOR_INITIALIZATION;
		}
	}
	
}



void initialization_process_full_init(){
	LF_init();
	RF_init();
	LB_init();
	RB_init();
	if(LeftFrontReach && LeftBackReach && RightFrontReach && RightBackReach){
		INIT_FLAG = 0;
		if(!ALL_TO_LIMIT_SWITCH){
			for(u8 i = 0; i < 4; i++)
				LiftingMotorPositionSetpoint[i] =  LiftingMotorBias[i] + TOTALLY_DOWN_SETPOINT;
		}
		else ALL_TO_LIMIT_SWITCH = 0;
		LeftFrontExpand = 1;
		RightFrontExpand = 1;
		RightBackExpand = 1;
		LeftBackExpand = 1;
		LeftFrontReach = 0;
		RightFrontReach = 0;
		RightBackReach = 0;
		LeftBackReach = 0;
	}


}



void initialization_process_front_init(){
	LF_init();
	RF_init();
}

void initialization_process_back_init(){
	LB_init();
	RB_init();

}


void LF_Dancing(int32_t ul, int32_t ll){
	if(CM1Encoder.ecd_angle - ul > -5000){
		LeftFrontReachUpper = 1;
	}
	if(ll - CM1Encoder.ecd_angle > -5000)
		LeftFrontReachLower = 1;
}

void RF_Dancing(int32_t ul, int32_t ll){
	if(CM2Encoder.ecd_angle - ul > -5000)
		RightFrontReachUpper = 1;
	if(ll - CM2Encoder.ecd_angle > -5000)
		RightFrontReachLower = 1;
}

void RB_Dancing(int32_t ul, int32_t ll){
	if(CM3Encoder.ecd_angle - ul > -5000)
		RightBackReachUpper = 1;
	if(ll - CM3Encoder.ecd_angle > -5000)
		RightBackReachLower = 1;
}

void LB_Dancing(int32_t ul, int32_t ll){
	if(CM4Encoder.ecd_angle - ul > -5000)
		LeftBackReachUpper = 1;
	if(ll - CM4Encoder.ecd_angle > -5000)
		LeftBackReachLower = 1;
}

void (*Dancing[4]) (int32_t, int32_t) = {LF_Dancing, RF_Dancing, RB_Dancing, LB_Dancing};

void clear()
{
    LeftFrontReachLower = 0;
		RightFrontReachLower = 0;
		RightBackReachLower = 0;
		LeftBackReachLower = 0;
    LeftFrontReachUpper = 0;
		RightFrontReachUpper = 0;
		RightBackReachUpper = 0;
		LeftBackReachUpper = 0;

}


void DancingMode(int32_t* ul, int32_t* ll)
{
	for(u8 i = 0; i < 4; i++)
		Dancing[i](ul[i], ll[i]);
	if(LeftFrontReachLower && RightFrontReachLower && RightBackReachLower && LeftBackReachLower){
		//only if all reaches lower limit will all lifting motors expand together
		for(u8 i = 0; i < 4; i++)
			LiftingMotorPositionSetpoint[i] = ul[i];
        clear();
			}
	if(LeftFrontReachUpper && RightFrontReachUpper && RightBackReachUpper && LeftBackReachUpper){
		//only if all reaches upper limit will all lifting motors go down together
		for(u8 i = 0; i < 4; i++)
			LiftingMotorPositionSetpoint[i] = ll[i];
		clear();		
	}
	
}

