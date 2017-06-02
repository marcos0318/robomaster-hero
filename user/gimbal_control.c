#include "gimbal_control.h"


//position control
float gimbalPositionSetpoint = 0;// prevGimbalPositionSetpoint = 0;
float bufferedGimbalPositionSetpoint = 0;
float gimbalPositionFeedback = 0;
bool isGimbalPositionSetpointIncrease = true;
struct fpid_control_states gimbalPositionState = {0,0,0};
int32_t yawPosMultiplier = 3;		//DBUS mouse yaw control

//velocity control
//struct inc_pid_states gimbalSpeedMoveState;// gimbalSpeedStaticState;
float gimbalSpeedSetpoint = 0;
float gimbalSpeedMoveOutput = 0;
float gimbalSpeedFeedback = 0;

struct fpid_control_states gimbalSpeedState = {0,0,0};

int32_t outsideLimit = 670;

int32_t turningConst = 2430;
//key control for chasis turning
bool KEY_Q_PREV = false;
bool KEY_E_PREV = false;

bool is_qe_turning = false;
/********************************/
/***** Gimbal Pitch Control *****/
/********************************/

//position control
float pitchPositionSetpoint = 0;// prevGimbalPositionSetpoint = 0;
float bufferedPitchPositionSetpoint = 0;
float pitchPositionFeedback= 0;
bool isPitchPositionSetpointIncrease = false;
int32_t storedPitch = 0;
struct fpid_control_states pitchPositionState = {0,0,0};

//velocity control
struct fpid_control_states pitchSpeedState = {0,0,0};
//struct inc_pid_states pitchSpeedMoveState;// gimbalSpeedStaticState;
float pitchSpeedSetpoint = 0;
float pitchSpeedFeedback = 0;
float pitchSpeedMoveOutput = 0;
int32_t pitchPosMultiplier = 3;       //DBUS mouse pitch control

float ytotalPrev = 0;
float rawpitchsetpoint = 0;

void keyboard_mouse_control() {
	xtotal =  DBUS_ReceiveData.mouse.xtotal;

	//move in the window
	if (abs(direction + output_angle*upperTotal / 3600) <= outsideLimit) 
		direction += (-DBUS_ReceiveData.rc.ch2 / 300 + -(xtotal - pre_xtotal)*14);

  //if is in the qe turnning state, just do not correct the direction
	//direction correction 
	if ( ! is_qe_turning ) { 
		if ((direction + output_angle*upperTotal / 3600) > outsideLimit)
			direction = outsideLimit - output_angle * upperTotal/3600;			
		else if ((direction + output_angle * upperTotal / 3600) < - outsideLimit)
			direction = -outsideLimit - output_angle * upperTotal / 3600;
	}
	
  //restart the gimbal, clear the setpoint of the gimbal
  if(ChasisFlag_Prev == 3 && (ChasisFlag == 1 || ChasisFlag == 2)) {
    direction = - output_angle*upperTotal/3600;
  }

  

	if(ChasisFlag == 1) {
    if(DBUS_CheckPush(KEY_Q) && !KEY_Q_PREV) {
				is_qe_turning = true;
        direction += turningConst;
		}
    if(DBUS_CheckPush(KEY_E) && !KEY_E_PREV) {
				is_qe_turning = true;
        direction -= turningConst;
		}
		setpoint_angle = -direction * 3600/upperTotal;
    gimbalPositionSetpoint = direction +  output_angle*upperTotal/3600;
		

	}
  else if (ChasisFlag == 2) {
    if(DBUS_CheckPush(KEY_Q) && !KEY_Q_PREV) {
      direction += turningConst;
      setpoint_angle -= turningConst * 3600 / upperTotal;
			is_qe_turning = true;
    }
    if(DBUS_CheckPush(KEY_E) && !KEY_E_PREV) {
      direction -= turningConst;
      setpoint_angle += turningConst * 3600 / upperTotal;
			is_qe_turning = true;
    }
    
    gimbalPositionSetpoint = direction +  output_angle*upperTotal/3600;

  }
  else if (ChasisFlag == 3) {
    gimbalPositionSetpoint = 0;
		setpoint_angle = output_angle;
    //The close of the gimbal is not here, but in the chasis control part
    //We directly bypass the setpoint_angle. 

    //Change setpoint angle here is not useful because there are not gimbal control

		/*
		if(DBUS_CheckPush(KEY_Q))
			setpoint_angle+=1;
		if(DBUS_CheckPush(KEY_E))
			setpoint_angle-=1;
		*/
  }
  else if (ChasisFlag == 4) {
    gimbalPositionSetpoint += -(xtotal - pre_xtotal)*14;
		/*
		if(DBUS_CheckPush(KEY_Q))
			setpoint_angle+=1;
		if(DBUS_CheckPush(KEY_E))
			setpoint_angle-=1;
		*/
  }

	//if is qe turning the gimbalsestpint is zero
	if (is_qe_turning) {
		gimbalPositionSetpoint = 0;
	}
  
	//Check gimbal flag
	
	
	if ( GimbalFlag == 2 || GimbalFlag == 1 ) {
		direction = - output_angle*upperTotal/3600;
	} 
	
	

	//Correction accorind to broke lines
	//Dynamic Detection
	if ( DBUSBrokenLineRecover ) {
		direction = - output_angle*upperTotal/3600;
		gimbalPositionSetpoint = 0;
	}
	if ( CAN1BrokenLineRecover ) {
		direction = - output_angle*upperTotal/3600;
		//setpoint_angle = -direction * 3600/upperTotal;
		gimbalPositionSetpoint = 0;
	}
	if ( CAN2BrokenLineRecover ) {
		direction = - output_angle*upperTotal/3600;
		//setpoint_angle = -direction * 3600/upperTotal;
		gimbalPositionSetpoint = 0;
	}

	//Static Detection
	if ( DBUSBrokenLine == 1 ) {
		direction = - output_angle*upperTotal/3600;
		gimbalPositionSetpoint = 0;
	}

	





	//Used for protection				
	if(gimbalPositionSetpoint > 900)
		gimbalPositionSetpoint = 900;
	else if (gimbalPositionSetpoint < -900)
		gimbalPositionSetpoint = -900;
  //Update data
	pre_xtotal = xtotal;
 	ChasisFlag_Prev = ChasisFlag;
	KEY_Q_PREV=DBUS_CheckPush(KEY_Q);
	KEY_E_PREV=DBUS_CheckPush(KEY_E);
	if ( abs(setpoint_angle - output_angle) < 100) {
		is_qe_turning = false;
	}
}

void gimbal_yaw_control(){

	isGimbalPositionSetpointIncrease = (bufferedGimbalPositionSetpoint < gimbalPositionSetpoint);

	if(isGimbalPositionSetpointIncrease){
		bufferedGimbalPositionSetpoint += 5;

		if (bufferedGimbalPositionSetpoint > gimbalPositionSetpoint)
			bufferedGimbalPositionSetpoint = gimbalPositionSetpoint;
	}
	else {
		bufferedGimbalPositionSetpoint -= 5;

		if(bufferedGimbalPositionSetpoint < gimbalPositionSetpoint) 
			bufferedGimbalPositionSetpoint = gimbalPositionSetpoint;
	}
	
	
	gimbalPositionFeedback = GMYawEncoder.ecd_angle;
	gimbalSpeedSetpoint = fpid_process(&gimbalPositionState, &bufferedGimbalPositionSetpoint, &gimbalPositionFeedback, kp_gimbalPosition, ki_gimbalPosition, kd_gimbalPosition);
	//Limit the output
	fwindowLimit(&gimbalSpeedSetpoint, 80, -80);
	
	//Get the speed here
	//incPIDsetpoint(&gimbalSpeedMoveState, gimbalSpeedSetpoint);
	//gimbalSpeedMoveOutput += incPIDcalc(&gimbalSpeedMoveState, GMYawEncoder.filter_rate);
	gimbalSpeedFeedback = GMYawEncoder.filter_rate;
	gimbalSpeedMoveOutput = fpid_process(&gimbalSpeedState, &gimbalSpeedSetpoint, &gimbalSpeedFeedback, 80, 4, 1 );
	// Direction Protection
	if ( DBUSBrokenLineRecover ) {
		fPIDClearError(&gimbalPositionState);
		fPIDClearError(&gimbalSpeedState);
		//incPIDClearError(&gimbalSpeedMoveState);
		gimbalSpeedMoveOutput = 0;
	}

	//Static Detection
	if ( DBUSBrokenLine == 1 || GimbalFlag == 1 || CAN1BrokenLine == 1 ) {
		fPIDClearError(&gimbalPositionState);
		fPIDClearError(&gimbalSpeedState);
		//incPIDClearError(&gimbalSpeedMoveState);
		gimbalSpeedMoveOutput = 0;
	}
}

void gimbal_pitch_control() {

	
	if (GimbalFlag == 2 || GimbalFlag == 1 ) {
		DBUS_ReceiveData.mouse.ytotal = 0;
	}
	//Dynamic Detection
	if (DBUSBrokenLineRecover) {
		DBUS_ReceiveData.mouse.ytotal = 0;
	}
	if (CAN2BrokenLineRecover) {
		DBUS_ReceiveData.mouse.ytotal = 0;
	}
	if (CAN1BrokenLineRecover) {
		DBUS_ReceiveData.mouse.ytotal = 0;
	}
	//Static Detection
	if ( DBUSBrokenLine == 1 ) {
		DBUS_ReceiveData.mouse.ytotal = 0;
	}



	//pitch setpoint



	rawpitchsetpoint +=  DBUS_ReceiveData.mouse.ytotal - ytotalPrev;
	rawpitchsetpoint +=  (float)DBUS_ReceiveData.rc.ch3 * 0.0007;

	//limit pitch position
	fwindowLimit(&rawpitchsetpoint, 1000/pitchPosMultiplier, 50/pitchPosMultiplier);


	pitchPositionSetpoint = -rawpitchsetpoint * pitchPosMultiplier;



	pitchPositionFeedback = GMPitchEncoder.ecd_angle;
	pitchSpeedSetpoint = fpid_process(&pitchPositionState, &pitchPositionSetpoint, &pitchPositionFeedback, kp_pitchPosition, ki_pitchPosition, kd_pitchPosition);
	
	fwindowLimit(&pitchSpeedSetpoint, 100, -100);
	
	pitchSpeedFeedback = GMPitchEncoder.filter_rate;
	pitchSpeedMoveOutput = fpid_process(&pitchSpeedState, &pitchSpeedSetpoint, &pitchSpeedFeedback, 80, 4, 1);


	//incPIDsetpoint(&pitchSpeedMoveState, pitchSpeedSetpoint);
	//pitchSpeedMoveOutput += incPIDcalc(&pitchSpeedMoveState, GMPitchEncoder.filter_rate);

	//Dynamic Detection
	if (DBUSBrokenLineRecover) {
		//incPIDClearError(&pitchSpeedMoveState);
		fPIDClearError(&pitchPositionState);
		fPIDClearError(&pitchSpeedState);

		pitchSpeedMoveOutput = 0;
	}

	//Static Detection
	if ( DBUSBrokenLine == 1 || GimbalFlag == 1 || CAN1BrokenLine == 1 ) {
		//incPIDClearError(&pitchSpeedMoveState);
		fPIDClearError(&pitchPositionState);
		fPIDClearError(&pitchSpeedState);
		pitchSpeedMoveOutput = 0;
	}

	ytotalPrev = DBUS_ReceiveData.mouse.ytotal; 

}



void TIM7_Int_Init(u16 period,u16 psc)//make timer interrupt by 1ms interrupt one time
{
    cameraPositionFeedback = GMCameraEncoder.ecd_angle;

    
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);
    
    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_Prescaler =psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
    TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    
    //TIM_SelectOutputTrigger(TIM3,TIM_TRGOSource_Update);
    /*—°‘Òupdate event◊˜Œ™TRGO,¿˚”√TIM3¥•∑¢ADCÕ®µ¿ */
    //√ø∏ˆ∂® ±÷‹∆⁄Ω· ¯∫Û¥•∑¢“ª¥Œ
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);
    TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM7, ENABLE);
    
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);
    /*œ»πÿ±’µ»¥˝ π”√*/
    
}


void TIM7_IRQHandler(void){
    
    if(TIM_GetITStatus(TIM7,TIM_IT_Update) != RESET)
    {
        
  					
//Used in control			
			//what ever the state is, these control functions are always running
			keyboard_mouse_control();
			//camera_position_control();
			gimbal_yaw_control();
			gimbal_pitch_control();
			GUN_PokeControl();
			if(CAN1BrokenLine == 0)
				Set_CM_Speed(CAN1, gimbalSpeedMoveOutput,pitchSpeedMoveOutput,gunSpeed,cameraSpeedOutput);
  		//if(CAN1BrokenLine_prev==0 && CAN1BrokenLine == 1)
					//Set_CM_Speed(CAN1,0,0,0,0);
  				
//Used in rest	
//						fPIDClearError(&gimbalPositionState);
//						fPIDClearError(&pitchPositionState);
//						//cameraPositionState
//						PIDClearError(&cameraSpeedState);
//						//pitchPositionState
//						incPIDClearError(&pitchSpeedMoveState);
//						//gimbalPositionState
//						incPIDClearError(&gimbalSpeedMoveState);
//						//gunPositionState
//						PIDClearError(&gunSpeedMoveState);
//						//direction = - output_angle*upperTotal/3600;
//						FRIC_SET_THRUST_L(0);
//						FRIC_SET_THRUST_R(0);
//						
//						Set_CM_Speed(CAN1, 0, 0, 0, 0);
				
				
			
				
    }
    TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
    
    
    
}






