
#include "GoOnStage.h"
static u32 ticks_msimg = 0;
volatile u32 TIM_7_counter = 0;
volatile uint8_t CAN2BrokenLine = 1;
uint8_t BROKEN_CABLE = 0;
volatile uint8_t INIT_FLAG = 1;
volatile uint8_t INIT_protection_up_stop_flag = 0;
volatile uint8_t INIT_protection_down_stop_flag = 0;
volatile uint8_t INIT_protection_up_begin_flag = 1;			//begin protection when turned on
volatile uint8_t INIT_protection_down_begin_flag = 0;
volatile u32 INIT_protection_timer_begin = 0;
volatile u32 INIT_protection_timer_reach = 0;
volatile u32 INIT_protection_timer_down = 0;
volatile uint8_t DANCING_MODE_FLAG = 0;
uint8_t INIT_FLAG_PREV = 1;
int32_t upper_limit[4] = {0}, lower_limit[4] = {0};
int32_t SPEED_LIMIT = 30000;

volatile uint32_t CAN2BrokenLineCounter = 0;
volatile uint32_t Wheel1BrokenLineCounter = 1;
volatile uint32_t Wheel2BrokenLineCounter = 1;
volatile uint32_t Wheel3BrokenLineCounter = 1;
volatile uint32_t Wheel4BrokenLineCounter = 1;



void readFeedback(){
      LiftingMotorSpeedFeedback[0] = CM1Encoder.filter_rate;
			LiftingMotorSpeedFeedback[1] = CM2Encoder.filter_rate;
			LiftingMotorSpeedFeedback[2] = CM3Encoder.filter_rate;
			LiftingMotorSpeedFeedback[3] = CM4Encoder.filter_rate;
			LiftingMotorPositionFeedback[0] = CM1Encoder.ecd_angle;
			LiftingMotorPositionFeedback[1] = CM2Encoder.ecd_angle;
			LiftingMotorPositionFeedback[2] = CM3Encoder.ecd_angle;
			LiftingMotorPositionFeedback[3] = CM4Encoder.ecd_angle;
}

void update_GPIO_state(){
	LeftFrontState[LeftFrontIndex] = gpio_read_input(LeftFront);
	LeftBackState[LeftBackIndex] = gpio_read_input(LeftBack);
	RightFrontState[RightFrontIndex] = gpio_read_input(RightFront);
	RightBackState[RightBackIndex] = gpio_read_input(RightBack);
	++LeftFrontIndex;
	++LeftBackIndex;
	++RightFrontIndex;
	++RightBackIndex;
	if(LeftFrontIndex >= READ_TIME)
		LeftFrontIndex = 0;
	if(LeftBackIndex >= READ_TIME)
		LeftBackIndex = 0;
	if(RightFrontIndex >= READ_TIME)
		RightFrontIndex = 0;
	if(RightBackIndex >= READ_TIME)
		RightBackIndex	 = 0;
}

uint8_t num_of_touch(const GPIO* gpio){
	uint8_t num = 0;
	if(gpio == LeftFront){
		for(uint8_t i = 0; i < READ_TIME; ++i){
			if(LeftFrontState[i] == 1)
				++num;
		}
		return num;
	}
	else if(gpio == LeftBack){
		for(uint8_t i = 0; i < READ_TIME; ++i){
			if(LeftBackState[i] == 1)
				++num;
		}
		return num;
	}
	else if(gpio == RightFront){
		for(uint8_t i = 0; i < READ_TIME; ++i){
			if(RightFrontState[i] == 1)
				++num;
		}
		return num;
	}
	else if(gpio == RightBack){
		for(uint8_t i = 0; i < READ_TIME; ++i){
			if(RightBackState[i] == 1)
				++num;
		}
		return num;
	}
}



void speedProcess(){
		//Control the cumulated error of the position pid process
		for (int i = 0; i<4; i++) {
			fpid_limit_cumulated_error(&LiftingMotorPositionState[i], 2000000);
		}
		
	
		//From position return the setpoint of the current speed
    for (uint8_t i=0;i<4;i++) {
        LiftingMotorSpeedSetpoint[i] = (int32_t)fpid_process(&LiftingMotorPositionState[i], &LiftingMotorPositionSetpoint[i], &LiftingMotorPositionFeedback[i],LMpos_kp,LMpos_ki,LMpos_kd );
			
    }
    

    
    //to correct the output to get a reasonable speed
    for(uint8_t i=0;i<4;i++){
        if(LiftingMotorSpeedSetpoint[i]>SPEED_SETPOINT_LIMIT) LiftingMotorSpeedSetpoint[i]=SPEED_SETPOINT_LIMIT;
        else if(LiftingMotorSpeedSetpoint[i]<-SPEED_SETPOINT_LIMIT) LiftingMotorSpeedSetpoint[i]=-SPEED_SETPOINT_LIMIT;
    }
    
    //to use buffered speed to do the real control 
    for (int i =0;i<4;i++){
        if (LiftingMotorSpeedSetpointBuffered[i]<LiftingMotorSpeedSetpoint[i]) LiftingMotorSpeedSetpointBuffered[i]+=5;
        else if (LiftingMotorSpeedSetpointBuffered[i]>LiftingMotorSpeedSetpoint[i]) LiftingMotorSpeedSetpointBuffered[i]-=5;
    }
    
    
    //to limit the buffered speed again. Ideally these lines are useless, but just add them to protect
    for(uint8_t i=0;i<4;i++){
        if(LiftingMotorSpeedSetpointBuffered[i]>SPEED_SETPOINT_LIMIT) LiftingMotorSpeedSetpointBuffered[i]=SPEED_SETPOINT_LIMIT;
        else if(LiftingMotorSpeedSetpointBuffered[i]<-SPEED_SETPOINT_LIMIT) LiftingMotorSpeedSetpointBuffered[i]=-SPEED_SETPOINT_LIMIT;
    }
    
		//Before pid process contrl the cummulated error 
		for (int i = 0; i < 4; i++) {
			pid_limit_cumulated_error(&LiftingMotorState[i], 10000);
		}
		
		
    //from the speed to get the current CM_Speed output
    for (uint8_t i = 0 ; i < 4 ; i++){
        LiftingMotorOutput[i] = pid_process(&LiftingMotorState[i], &LiftingMotorSpeedSetpointBuffered[i], &LiftingMotorSpeedFeedback[i], kp,ki,kd);
    }
    
		//limit the CM_Speed output to 30000, which is reasonable here. But this number could be adjust later on 
    for (uint8_t i = 0 ; i < 4 ; i++){
        if (LiftingMotorOutput[i]>SPEED_LIMIT) LiftingMotorOutput[i]=SPEED_LIMIT;
        if (LiftingMotorOutput[i]<-SPEED_LIMIT) LiftingMotorOutput[i]=-SPEED_LIMIT;
    }
		
		Set_CM_Speed(CAN2,LiftingMotorOutput[0],LiftingMotorOutput[1],LiftingMotorOutput[2],LiftingMotorOutput[3]);
    
}

void setSetpoint(){
    if(BREAK && GO_ON_STAGE_ONE_KEY) GO_ON_STAGE_ONE_KEY=false;
    if(BREAK && GO_DOWN_STAGE_ONE_KEY) GO_DOWN_STAGE_ONE_KEY=false;
    if(!GO_ON_STAGE_ONE_KEY_PREV && GO_ON_STAGE_ONE_KEY) {
        //ticks_msimg_on_prev=ticks_msimg;			//starting counting time for the procedure
        for(uint8_t i=0;i<4;i++)
            LiftingMotorPositionSetpoint[i] = LiftingMotorBias[i] + UP_SETPOINT;
        //may need to be loaded into flash memory when testing using the RC
        GO_ON_STAGE_ONE_KEY=false;
    }
    if(!GO_DOWN_STAGE_ONE_KEY_PREV && GO_DOWN_STAGE_ONE_KEY) {
        //ticks_msimg_down_prev=ticks_msimg;			//starting counting time for the procedure
        LiftingMotorPositionSetpoint[2]=LiftingMotorBias[2]+MID_SETPOINT;
				LiftingMotorPositionSetpoint[3]=LiftingMotorBias[3]+MID_SETPOINT;
        LiftingMotorPositionSetpoint[0]=LiftingMotorBias[0]+UP_SETPOINT;
				LiftingMotorPositionSetpoint[1]=LiftingMotorBias[1]+UP_SETPOINT;
        GO_DOWN_STAGE_ONE_KEY=false;
    }
    if(ONE_KEY_UP_FRONT){
			LiftingMotorPositionSetpoint[0]=LiftingMotorBias[0]+UP_SETPOINT;
			LiftingMotorPositionSetpoint[1]=LiftingMotorBias[1]+UP_SETPOINT;
		}
    else if(ONE_KEY_DOWN_FRONT){
			LiftingMotorPositionSetpoint[0]=LiftingMotorBias[0]+DOWN_SETPOINT;
			LiftingMotorPositionSetpoint[1]=LiftingMotorBias[1]+DOWN_SETPOINT;
		}
    if(ONE_KEY_UP_BACK){
			LiftingMotorPositionSetpoint[2]=LiftingMotorBias[2]+UP_SETPOINT;
			LiftingMotorPositionSetpoint[3]=LiftingMotorBias[3]+UP_SETPOINT;
		}
    else if(ONE_KEY_DOWN_BACK){
			LiftingMotorPositionSetpoint[2]=LiftingMotorBias[2]+DOWN_SETPOINT;
			LiftingMotorPositionSetpoint[3]=LiftingMotorBias[3]+DOWN_SETPOINT;
		}
				
    
    GO_ON_STAGE_ONE_KEY_PREV=GO_ON_STAGE_ONE_KEY;			//don't forget to update
    GO_DOWN_STAGE_ONE_KEY_PREV=GO_DOWN_STAGE_ONE_KEY;			//don't forget to update
    

}


void TIM7_Int_Init(u16 period,u16 psc)//make timer interrupt by 1ms interrupt one time
{
  

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);
    
    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_Prescaler =psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
    TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    
    //TIM_SelectOutputTrigger(TIM3,TIM_TRGOSource_Update);
    /*—°‘Òupdate event?˜Œ™TRGO,¿°”vTIM3¥•?¢ADCÕ®µ¿ */
    //vø?ˆ?® ±÷‹?/O· ¯?Û¥•?¢“ª¥Œ
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
    /*œ»pÿ±’µ»¥? p”v*/
    
}


void TIM7_IRQHandler(void){
    
    if(TIM_GetITStatus(TIM7,TIM_IT_Update)!=RESET)
    {
				++TIM_7_counter;
        ticks_msimg = get_ms_ticks();
				update_GPIO_state();	//always update the GPIO state array
				if(TIM_7_counter % 20 == 0 )
				{
					CAN2BrokenLine = checkBrokenLine(TIM_7_counter, Wheel1BrokenLineCounter)
											|| checkBrokenLine(TIM_7_counter, Wheel2BrokenLineCounter)
											|| checkBrokenLine(TIM_7_counter, Wheel3BrokenLineCounter)
											|| checkBrokenLine(TIM_7_counter, Wheel4BrokenLineCounter);
				}
			/*
			To do here:
			if not in init state, still need to detect
			if touch the GPIO switch for a long time
			then unconditionally stop
			And I will need to update the bias, lower_limit and upper_limit
			*/
			
			
			//friction wheel initialization needs to be delayed
			if(TIM_7_counter == 3000)
				Friction_wheel_init();

			if(TIM_7_counter == 5000)
			  GUN_Init();
				
			if(TIM_7_counter>10000){
				if(!FRICTION_WHEEL_STATE)
					friction_wheel_setpoint=0;
				else {
					if(friction_wheel_setpoint < 350)
						friction_wheel_setpoint+=1;
				}
				FRIC_SET_THRUST_L(friction_wheel_setpoint);
				FRIC_SET_THRUST_R(friction_wheel_setpoint);
				FRIC_SET_THRUST_M(friction_wheel_setpoint);
			}
			
			
				if(INIT_FLAG){
						SPEED_LIMIT = 30000;
						initialization_process_full_init();
					
				}
				if(!INIT_FLAG){
					SPEED_LIMIT = 30000;
					if(DANCING_MODE_FLAG){
						for(u8 i = 0; i < 4; i++)
						{
							//lower_limit[i]=LiftingMotorPositionLimit[i] - DANCING_MODE_UP_DOWN_DIFF;
							//upper_limit[i]=LiftingMotorPositionLimit[i] - DOWN_SETPOINT;
							upper_limit[i]=LiftingMotorBias[i] + DANCING_MODE_RASING_HEIGHT;
							lower_limit[i]=upper_limit[i] - DANCING_MODE_UP_DOWN_DIFF;
						}
						DancingMode(upper_limit, lower_limit);
					}
					if(!DANCING_MODE_FLAG)
						setSetpoint();
				}
				//INIT time protection
			if( !BROKEN_CABLE && !CAN2BrokenLine){
				if(INIT_protection_up_begin_flag && !HAS_ALL_REACHED_FLAG && ((TIM_7_counter-INIT_protection_timer_begin)>INIT_UP_PROTECTION_TIME) )
				{
					//has already raising for the maximum raising time, still not all reached 
					LiftingMotorPositionLimit[0] = CM1Encoder.ecd_angle;
					LiftingMotorBias[0] = LiftingMotorPositionLimit[0]  - UP_DOWN_DISTANCE;
//					LiftingMotorUpperLimit[0] = LiftingMotorBias[0] + UP_SETPOINT;
					LiftingMotorPositionSetpoint[0] = CM1Encoder.ecd_angle - DOWN_SETPOINT;
					LiftingMotorPositionLimit[1] = CM2Encoder.ecd_angle;
					LiftingMotorBias[1] = LiftingMotorPositionLimit[1]  - UP_DOWN_DISTANCE;
//					LiftingMotorUpperLimit[1] = LiftingMotorBias[1] + UP_SETPOINT;
					LiftingMotorPositionSetpoint[1] = CM2Encoder.ecd_angle - DOWN_SETPOINT;
					LiftingMotorPositionLimit[2] = CM3Encoder.ecd_angle;
					LiftingMotorBias[2] = LiftingMotorPositionLimit[2]  - UP_DOWN_DISTANCE;
//					LiftingMotorUpperLimit[2] = LiftingMotorBias[2] + UP_SETPOINT;
					LiftingMotorPositionSetpoint[2] = CM3Encoder.ecd_angle - DOWN_SETPOINT;
					LiftingMotorPositionLimit[3] = CM3Encoder.ecd_angle;
					LiftingMotorBias[3] = LiftingMotorPositionLimit[2]  - UP_DOWN_DISTANCE;
//					LiftingMotorUpperLimit[3] = LiftingMotorBias[2] + UP_SETPOINT;
					LiftingMotorPositionSetpoint[3] = CM3Encoder.ecd_angle - DOWN_SETPOINT;
					
					INIT_protection_up_begin_flag = 0;
					HAS_ALL_REACHED_FLAG = 0;
					INIT_FLAG = 0;
					
				}
				else if(INIT_protection_up_begin_flag && HAS_ALL_REACHED_FLAG && ((TIM_7_counter-INIT_protection_timer_begin)<=INIT_UP_PROTECTION_TIME) ){
					INIT_protection_up_begin_flag = 0;
					HAS_ALL_REACHED_FLAG = 0;
				}
				if(!INIT_protection_up_begin_flag){
					HAS_ALL_REACHED_FLAG = 0;
				}
				
				if(INIT_protection_down_begin_flag && !TP_reach_lower_detection() && ((TIM_7_counter-INIT_protection_timer_reach)>INIT_DOWN_PROTECTION_TIME))
				{
					LiftingMotorBias[0] = LiftingMotorPositionSetpoint[0] = CM1Encoder.ecd_angle;
					LiftingMotorBias[1] = LiftingMotorPositionSetpoint[1] = CM2Encoder.ecd_angle;
					LiftingMotorBias[2] = LiftingMotorPositionSetpoint[2] = CM3Encoder.ecd_angle;
					LiftingMotorBias[3] = LiftingMotorPositionSetpoint[3] = CM4Encoder.ecd_angle;	
					for(u8 i = 0; i < 4; i++)
					{
//						LiftingMotorUpperLimit[i] = LiftingMotorBias[i] + UP_SETPOINT;
						LiftingMotorPositionLimit[i] = LiftingMotorBias[i] + UP_DOWN_DISTANCE;
					}
					INIT_protection_down_begin_flag = 0;
				}
				else if(INIT_protection_down_begin_flag && TP_reach_lower_detection()){
					INIT_protection_timer_down = TIM_7_counter;
					INIT_protection_down_begin_flag = 0;
				}
			}
				broken_time=ticks_msimg;
				if((broken_time-receive_time)>200 || CAN2BrokenLine)
				{
					if((broken_time-receive_time)>200)
						BROKEN_CABLE=1;
					else BROKEN_CABLE = 0;
					INIT_protection_up_begin_flag = 1;
					INIT_FLAG = 1;
					INIT_protection_timer_begin = TIM_7_counter;
					for(uint8_t i=0; i<4; i++)
						PIDClearError(&LiftingMotorState[i]);
					Set_CM_Speed(CAN2,0,0,0,0);
				}
				else{
					BROKEN_CABLE=0;
					speedProcess();
				}
			
			INIT_FLAG_PREV = INIT_FLAG;
				
    }
    TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
    
    
    
}



