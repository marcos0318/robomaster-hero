#include "initialization_process.h"
#include "GoOnStage.h"
uint8_t LeftFrontExpand = 0;
uint8_t RightFrontExpand = 0;
uint8_t RightBackExpand = 0;
uint8_t LeftBackExpand = 0;
uint8_t LeftFrontReach = 0;
uint8_t RightFrontReach = 0;
uint8_t RightBackReach = 0;
uint8_t LeftBackReach = 0;




void Limit_Switch_init(){
	
	BUTTON_init(LeftFront);
	BUTTON_init(LeftBack);
	BUTTON_init(RightFront);
	BUTTON_init(RightBack);

}

/*
volatile int32_t LiftingMotorPositionSetpoint[4]={0};
int32_t LiftingMotorBias[4] = {0};		//actually its the lower limit
int32_t LiftingMotorUpperLimit[4] = {UP_SETPOINT, UP_SETPOINT, UP_SETPOINT, UP_SETPOINT};
int32_t LiftingMotorPositionLimit[4] = {UP_DOWN_DISTANCE, UP_DOWN_DISTANCE, UP_DOWN_DISTANCE, UP_DOWN_DISTANCE};

#define TOTALLY_DOWN_SETPOINT 1000
#define UP_SETPOINT 255000						//determined by the height of the pneumatic, where pneumatice can be put on the stage precisely
#define DOWN_SETPOINT 1000//determined by the relative height between the pneumatic and the wheels, whe wheels should be put on the stage precisely
#define UP_DOWN_DISTANCE 280000

if(LiftingMotorPositionSetpoint[i] > LiftingMotorUpperLimit[i]){
			LiftingMotorUpperLimit[i]=LiftingMotorPositionSetpoint[i];
			LiftingMotorBias[i]=LiftingMotorUpperLimit[i]-UP_SETPOINT;
	}


LiftingMotorPositionFeedback[0] = CM1Encoder.ecd_angle;
			LiftingMotorPositionFeedback[1] = CM2Encoder.ecd_angle;
			LiftingMotorPositionFeedback[2] = CM3Encoder.ecd_angle;
			LiftingMotorPositionFeedback[3] = CM4Encoder.ecd_angle;
*/


void LF_init(){
	if(LeftFrontExpand){
		if(num_of_touch(LeftFront)>30){
			LeftFrontExpand = 0;
			LeftFrontReach = 1;
			LiftingMotorPositionLimit[0] = CM1Encoder.ecd_angle;
			LiftingMotorBias[0] = LiftingMotorPositionLimit[0] - UP_DOWN_DISTANCE;
			LiftingMotorUpperLimit[0] = LiftingMotorBias[0] + UP_SETPOINT;
			LiftingMotorPositionSetpoint[0] = LiftingMotorBias[0] + TOTALLY_DOWN_SETPOINT;
	
		}
		else {
			LiftingMotorPositionSetpoint[0] = LiftingMotorBias[0] + RASING_HEIGHT_FOR_INITIALIZATION;
		}
	}
	//if(LeftFrontReach){}

}
void RF_init(){
	if(RightFrontExpand){
		if(num_of_touch(RightFront)>30){
			RightFrontExpand = 0;
			RightFrontReach = 1;
			LiftingMotorPositionLimit[1] = CM2Encoder.ecd_angle;
			LiftingMotorBias[1] = LiftingMotorPositionLimit[1]  - UP_DOWN_DISTANCE;
			LiftingMotorUpperLimit[1] = LiftingMotorBias[1] + UP_SETPOINT;
			LiftingMotorPositionSetpoint[1] = LiftingMotorBias[1] + TOTALLY_DOWN_SETPOINT;
			
		}
		else {
			LiftingMotorPositionSetpoint[1] = LiftingMotorBias[1] + RASING_HEIGHT_FOR_INITIALIZATION;
		}
	}


}

void LB_init(){
	if(LeftBackExpand){
		if(num_of_touch(LeftBack)>30){
			LeftFrontExpand = 0;
			LeftFrontReach = 1;
			LiftingMotorPositionLimit[3] = CM4Encoder.ecd_angle;
			LiftingMotorBias[3] = LiftingMotorPositionLimit[3] - UP_DOWN_DISTANCE;
			LiftingMotorUpperLimit[3] = LiftingMotorBias[3] + UP_SETPOINT;
			LiftingMotorPositionSetpoint[3] = LiftingMotorBias[3] + TOTALLY_DOWN_SETPOINT;
	
		}
		else {
			LiftingMotorPositionSetpoint[3] = LiftingMotorBias[3] + RASING_HEIGHT_FOR_INITIALIZATION;
		}
	}
	
}

void RB_init(){
	if(RightBackExpand){
		if(num_of_touch(RightBack)>30){
			RightBackExpand = 0;
			RightBackReach = 1;
			LiftingMotorPositionLimit[2] = CM3Encoder.ecd_angle;
			LiftingMotorBias[2] = LiftingMotorPositionLimit[2]  - UP_DOWN_DISTANCE;
			LiftingMotorUpperLimit[2] = LiftingMotorBias[2] + UP_SETPOINT;
			LiftingMotorPositionSetpoint[2] = LiftingMotorBias[2] + TOTALLY_DOWN_SETPOINT;
			
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
		LeftFrontExpand = 0;
		RightFrontExpand = 0;
		RightBackExpand = 0;
		LeftBackExpand = 0;
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



