#include "gimbal_control.h"

volatile u32 TIM_7_Counter = 0;

u8 FOR_JOHN_SHIFT_G_SPECIAL_MODE = 0;
u8 SHIFT_F_F_up_state = 0;
u8 FOR_JOHN_G_PREV = 0;
u8 FOR_JOHN_F_PREV = 0;
u8 FOR_JOHN_SHIFT_G_PREV = 0;
u8 FOR_JOHN_SHIFT_F_PREV = 0;
u8 FOR_JOHN_SHIFT_F = 0;
u8 FOR_JOHN_SHIFT_G = 0;
u8 FOR_JOHN_F = 0;
u8 FOR_JOHN_G = 0;
u8 SHIFT_F_F_DETECTOR = 0;
u8 DO_NOT_SAVE_TIME_SHIFT_F = 0;
u8 DO_NOT_SAVE_TIME_SHIFT_G = 0;
u8 SHIFT_G_G_DETECTOR = 0;
volatile u32 SHIFT_G_timer = 0;
volatile u32 SHIFT_F_timer = 0;
//position control
float gimbalPositionSetpoint = 0;// prevGimbalPositionSetpoint = 0;
float bufferedGimbalPositionSetpoint = 0;
float gimbalPositionFeedback = 0;
bool isGimbalPositionSetpointIncrease = true;
struct fpid_control_states gimbalPositionState = {0,0,0};
int32_t yawPosMultiplier = 3;		//DBUS mouse yaw control
u8 LeftJoystick = 1;
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
	if(LeftJoystick)
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
        
			++TIM_7_Counter;
  					
//Used in control			
			//what ever the state is, these control functions are always running
			keyboard_mouse_control();
			//camera_position_control();
			gimbal_yaw_control();
			gimbal_pitch_control();
			GUN_PokeControl();
			if(CAN1BrokenLine == 0)
				Set_CM_Speed(CAN1, gimbalSpeedMoveOutput,pitchSpeedMoveOutput,gunSpeed,cameraSpeedOutput);
				
				if(TIM_7_Counter % 20 == 10){
				//key check
				FOR_JOHN_G=DBUS_CheckPush(KEY_G);
				FOR_JOHN_F=DBUS_CheckPush(KEY_F);
				FOR_JOHN_SHIFT_F = DBUS_CheckPush(KEY_F) && DBUS_CheckPush(KEY_SHIFT);
				FOR_JOHN_SHIFT_G = DBUS_CheckPush(KEY_G) && DBUS_CheckPush(KEY_SHIFT);
				
				if(!DBUS_CheckPush(KEY_CTRL) && SHIFT_F_F_DETECTOR &&(TIM_7_Counter - SHIFT_F_timer) > 500) {
					SHIFT_F_F_DETECTOR = 0;
					//SHIFT+F
						HERO = RUNNING_MODE;
						switch_and_send();
				}
				if(!DBUS_CheckPush(KEY_CTRL) && SHIFT_G_G_DETECTOR && (TIM_7_Counter - SHIFT_G_timer) > 500) {
					SHIFT_G_G_DETECTOR = 0;
					//SHIFT+G
						lower_pneumatic_state = 1;
						pneumatic_control(1, 1);
						pneumatic_control(2, 1);	
						//jump to a special mode, after that mode, if press G, will jump to SPEED_LIMITATION
						LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = DOWN_SETPOINT/8;
						DataMonitor_Send(71, 0);		//ONE_KEY_DOWN_FRONT
						//reverse QWEASD
						filter_rate_limit = FOR_JOHN_INTO_RI_MAX_SPEED;
						speed_multiplier = -FOR_JOHN_INTO_RI_MAX_SPEED;
						//turn off gyro
						ChasisFlag = 3;	
						lower_pneumatic_state = 1;
						pneumatic_control(1, 1);
						pneumatic_control(2, 1);					
						FOR_JOHN_SHIFT_G_SPECIAL_MODE = 1;
					
				}
				if(!DBUS_CheckPush(KEY_CTRL) && !FOR_JOHN_SHIFT_F && FOR_JOHN_SHIFT_F_PREV){
					
					if(!DO_NOT_SAVE_TIME_SHIFT_F){
						SHIFT_F_F_DETECTOR = 1;
						SHIFT_F_timer = TIM_7_Counter;
						DO_NOT_SAVE_TIME_SHIFT_F = 0;
					}
					else {
						DO_NOT_SAVE_TIME_SHIFT_F = 0;
						SHIFT_F_F_DETECTOR = 0;
					}
				}
				if(!DBUS_CheckPush(KEY_CTRL) && !FOR_JOHN_SHIFT_G && FOR_JOHN_SHIFT_G_PREV){
					
					if(!DO_NOT_SAVE_TIME_SHIFT_G){
						SHIFT_G_timer = TIM_7_Counter;
						DO_NOT_SAVE_TIME_SHIFT_G = 0;
						SHIFT_G_G_DETECTOR = 1;
					}
					else {
						SHIFT_G_G_DETECTOR = 0;
						DO_NOT_SAVE_TIME_SHIFT_G = 0;
					}
				}
				if(!DBUS_CheckPush(KEY_CTRL) && FOR_JOHN_SHIFT_F && !FOR_JOHN_SHIFT_F_PREV && ((TIM_7_Counter - SHIFT_F_timer) < 500)){
					
					//SHIFT+F+F
						//upper pneumatic extends
						if(SHIFT_F_F_up_state == 0){
							pneumatic_control(3,1);
							SHIFT_F_F_up_state =1;
						}
						else{
							pneumatic_control(3,0);
							SHIFT_F_F_up_state =0;
						}
					
					SHIFT_F_F_DETECTOR = 0;
						DO_NOT_SAVE_TIME_SHIFT_F = 1;
				}
				
				if(!DBUS_CheckPush(KEY_CTRL) && FOR_JOHN_SHIFT_G && !FOR_JOHN_SHIFT_G_PREV && ((TIM_7_Counter - SHIFT_G_timer) < 500)){
					
						//SHIFT+G+G
						//jump to jump off island mode
					SHIFT_G_G_DETECTOR = 0;
						pneumatic_control(1, 0);
						pneumatic_control(2, 0);
						HERO = DOWN_FRONT_WHEEL;
					switch_and_send();
					DO_NOT_SAVE_TIME_SHIFT_G = 1;
					
				}
				if(!DBUS_CheckPush(KEY_CTRL) && FOR_JOHN_SHIFT_G_SPECIAL_MODE && !FOR_JOHN_G_PREV && FOR_JOHN_G)
				{
						HERO = SPEED_LIMITATION;
						switch_and_send();
						FOR_JOHN_SHIFT_G_SPECIAL_MODE = 0;
				}

				//key check
				FOR_JOHN_G_PREV=DBUS_CheckPush(KEY_G);
				FOR_JOHN_F_PREV=DBUS_CheckPush(KEY_F);
				FOR_JOHN_SHIFT_F_PREV = DBUS_CheckPush(KEY_F) && DBUS_CheckPush(KEY_SHIFT);
				FOR_JOHN_SHIFT_G_PREV = DBUS_CheckPush(KEY_G) && DBUS_CheckPush(KEY_SHIFT);
			}
				
				
				
				//INTO_RI lower pneumatic delay extention
				if(INTO_RI_LPneu_flag == 1 && ((TIM_7_Counter - INTO_RI_LPneu_timer) > 3000))
				{
					INTO_RI_LPneu_flag = 0;
					lower_pneumatic_state=true;
					pneumatic_control(1, 1);
					pneumatic_control(2, 1);
				}
				//SPEED_LIMITATION lower pneumatic delay withdrawl
				if(SPEED_LIMITATION_LPneu_flag == 1 && ((TIM_7_Counter - SPEED_LIMITATION_LPneu_timer) > 2000))
				{
					SPEED_LIMITATION_LPneu_flag = 0;
					lower_pneumatic_state=false;
					pneumatic_control(1, 0);
					pneumatic_control(2, 0);
				}
				//VERTICAL_PNEUMATIC_WITHDRAWS upper horizontal pneumatic delay withdrawl, LiftingMotors delay withdrawal
				if(VERTICAL_PNEUMATIC_WITHDRAWS_UHPneu_LM_flag == 1 && ((TIM_7_Counter - VERTICAL_PNEUMATIC_WITHDRAWS_UHPneu_LM_timer) > 2000))
				{
					VERTICAL_PNEUMATIC_WITHDRAWS_UHPneu_LM_flag = 0;
					pneumatic_control(4, false);
					upper_pneumatic_state = 1;
					pneumatic_control(3, false);
					//Lifting Motors go down
					LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
					DataMonitor_Send(5, 0);
				}
				//Back_To_RUNNING_MODE: upper horizontal pneumatic delay withdrawl, LiftingMotors delay withdrawal
				if(B_RUNNING_MODE_UHPneu_LM_flag == 1 && ((TIM_7_Counter - B_RUNNING_MODE_UHPneu_LM_timer) > 2000))
				{
					B_RUNNING_MODE_UHPneu_LM_flag = 0;
					pneumatic_control(4, 0);
					upper_pneumatic_state = 1;
					pneumatic_control(3, 0);
					//Lifting Motors go down
					LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
					DataMonitor_Send(5, 0);
				}
				
				//Back_To_DANCING_MODE: upper vertical pneumatic delay extension, friction wheels delay being turned on
				if(B_DANCING_MODE_UVPneu_FW_flag == 1 && ((TIM_7_Counter - B_DANCING_MODE_UVPneu_FW_timer) > 2000))
				{
					B_DANCING_MODE_UVPneu_FW_flag = 0;
					pneumatic_control(4, true);
					DataMonitor_Send(63, 0);
				}
  		
    }
    TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
    
    
    
}






