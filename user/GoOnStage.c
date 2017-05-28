
#include "GoOnStage.h"
static u32 ticks_msimg = 0;
uint8_t BROKEN_CABLE = 0;
volatile uint8_t INIT_FLAG = 1;
uint8_t INIT_FLAG_PREV = 1;
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
    for(uint8_t i=0;i<4;i++){
        LiftingMotorSpeedSetpoint[i] = (int32_t)fpid_process(&LiftingMotorPositionState[i], &LiftingMotorPositionSetpoint[i], &LiftingMotorPositionFeedback[i],LMpos_kp,LMpos_ki,LMpos_kd );
        
    }
    
    
    
    
    for(uint8_t i=0;i<4;i++){
        if(LiftingMotorSpeedSetpoint[i]>SPEED_SETPOINT_LIMIT) LiftingMotorSpeedSetpoint[i]=SPEED_SETPOINT_LIMIT;
        else if(LiftingMotorSpeedSetpoint[i]<-SPEED_SETPOINT_LIMIT) LiftingMotorSpeedSetpoint[i]=-SPEED_SETPOINT_LIMIT;
    }
    
    
    for (int i =0;i<4;i++){
        if (LiftingMotorSpeedSetpointBuffered[i]<LiftingMotorSpeedSetpoint[i]) LiftingMotorSpeedSetpointBuffered[i]+=5;
        else if (LiftingMotorSpeedSetpointBuffered[i]>LiftingMotorSpeedSetpoint[i]) LiftingMotorSpeedSetpointBuffered[i]-=5;
    }
    
    
    
    for(uint8_t i=0;i<4;i++){
        if(LiftingMotorSpeedSetpointBuffered[i]>SPEED_SETPOINT_LIMIT) LiftingMotorSpeedSetpointBuffered[i]=SPEED_SETPOINT_LIMIT;
        else if(LiftingMotorSpeedSetpointBuffered[i]<-SPEED_SETPOINT_LIMIT) LiftingMotorSpeedSetpointBuffered[i]=-SPEED_SETPOINT_LIMIT;
    }
    
    
    for (uint8_t i = 0 ; i < 4 ; i++){
        LiftingMotorOutput[i] = pid_process(&LiftingMotorState[i], &LiftingMotorSpeedSetpointBuffered[i], &LiftingMotorSpeedFeedback[i], kp,ki,kd);
    }
    
    for (uint8_t i = 0 ; i < 4 ; i++){
        if (LiftingMotorOutput[i]>30000) LiftingMotorOutput[i]=30000;
        if (LiftingMotorOutput[i]<-30000) LiftingMotorOutput[i]=-30000;
    }
		
		Set_CM_Speed(CAN2,LiftingMotorOutput[0],LiftingMotorOutput[1],LiftingMotorOutput[2],LiftingMotorOutput[3]);
    
}

void setSetpoint(){
    if(BREAK && GO_ON_STAGE_ONE_KEY) GO_ON_STAGE_ONE_KEY=false;
    if(BREAK && GO_DOWN_STAGE_ONE_KEY) GO_DOWN_STAGE_ONE_KEY=false;
    if(!GO_ON_STAGE_ONE_KEY_PREV && GO_ON_STAGE_ONE_KEY) {
        //ticks_msimg_on_prev=ticks_msimg;			//starting counting time for the procedure
        for(uint8_t i=0;i<4;i++)
            LiftingMotorPositionSetpoint[i]=LiftingMotorBias[i]+UP_SETPOINT;
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
        ticks_msimg = get_ms_ticks();
				update_GPIO_state();	//always update the GPIO state array
			/*
			To do here:
			if not in init state, still need to detect
			if touch the GPIO switch for a long time
			then unconditionally stop
			And I will need to update the bias, lower_limit and upeer_limit
			*/
				if(INIT_FLAG){
					
						initialization_process_full_init();
					
				}
				if(!INIT_FLAG)
					setSetpoint();
				broken_time=ticks_msimg;
				if((broken_time-receive_time)>200)
				{
					BROKEN_CABLE=1;
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



