#include "BilateralCommunication.h"
static DMA_InitTypeDef Bilateral_DMA_InitStruct;
#define BilateralCommunication_buffer 3
int8_t BilateralBuffer[BilateralCommunication_buffer];
u32 receive_time=0;
u32 broken_time=0;
u8 HAS_RECEIVED_LOAD = 0;
void Bilateral_Init(void) {
	
	
    DMA_InitTypeDef     DMA_InitStructure;
    NVIC_InitTypeDef	NVIC_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
    GPIO_InitTypeDef	GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
    
		
		USART_InitTypeDef   USART_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
    USART_InitStructure.USART_BaudRate              =   115200;
    USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  =   USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity                =   USART_Parity_No;
    USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
    USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStructure);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    USART_Cmd(USART3, ENABLE);
		
		    //USART3
		NVIC_InitStructure.NVIC_IRQChannel						=	USART3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	8;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
		NVIC_Init(&NVIC_InitStructure);
		USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
			
		  //UART3_RX
    DMA_InitStructure.DMA_Channel           =   DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr=   (uint32_t)(&USART3->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)(BilateralBuffer);   
    DMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize        =   BilateralCommunication_buffer;	//now its 3
    DMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream1, ENABLE);
		
}

uint8_t getID(){
	return BilateralBuffer[0];
}

int16_t getPositionSetpoint(){
	return (BilateralBuffer[1]<<8)+BilateralBuffer[2];
	
}

void modifyingBias(uint8_t i, u16 step){
	LiftingMotorPositionSetpoint[i]-=step;
	if(LiftingMotorPositionSetpoint[i] < LiftingMotorBias[i]){
			//LiftingMotorBias[i]=LiftingMotorPositionSetpoint[i];
			//LiftingMotorPositionLimit[i] = LiftingMotorBias[i] + UP_DOWN_DISTANCE;
		LiftingMotorPositionSetpoint[i] = LiftingMotorBias[i];
	}
}

void modifyingUpperLimit(uint8_t i, u16 step){
	LiftingMotorPositionSetpoint[i]+=step;
	if(LiftingMotorPositionSetpoint[i] > LiftingMotorPositionLimit[i]){
			//LiftingMotorPositionLimit[i] = LiftingMotorPositionSetpoint[i];
			//LiftingMotorBias[i]=LiftingMotorPositionLimit[i]-UP_DOWN_DISTANCE;
		 LiftingMotorPositionSetpoint[i] = LiftingMotorPositionLimit[i];
	}
}
u8 UARTtemp1;
void USART3_IRQHandler(void)
{
    UARTtemp1 = USART3->DR;
    UARTtemp1 = USART3->SR;
    
    DMA_Cmd(DMA1_Stream1, DISABLE);
	
	
	
		
		ONE_KEY_DOWN_FRONT=false;
		ONE_KEY_UP_FRONT=false;
		ONE_KEY_DOWN_BACK=false;
		ONE_KEY_UP_BACK=false;
		BREAK=false;
		if(getID() == 90){
			INIT_FLAG = 1;
			INIT_protection_up_begin_flag = 1;
			INIT_protection_timer_begin = TIM_7_counter;
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID() == 72) {
			//load UP_SETPOINT
			if(getPositionSetpoint() == 2)
			{
				//load DANCING_MODE_RAISING_HEIGHT
				Set_CM_Speed(CAN2, 0, 0, 0, 0);
				BSP_DWT_DelayMs(1000);
				//delay 1s
				FLASH_MEM[1] = (u32)(CM3Encoder.ecd_angle - LiftingMotorBias[2]);
				writeFlash(FLASH_MEM, 2);
				
			}
			HAS_RECEIVED_LOAD = 1;
			for(u8 i = 0;i <4; i++)
				LiftingMotorPositionSetpoint[i] = LiftingMotorBias[i] + FLASH_MEM[1];
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==0x05){
			//shift R
			//all go to down limit
			DANCING_MODE_FLAG = 0;
			for(uint8_t i = 0; i < 4; i++)
				LiftingMotorPositionSetpoint[i]=LiftingMotorBias[i];
			FRICTION_WHEEL_STATE = false;
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==69){
			//all go up to the limit switch
			//ALL_TO_LIMIT_SWITCH = 1;
			//INIT_FLAG = 1;
			for(u8 i = 0; i < 4; i++)
								//LiftingMotorPositionSetpoint[i] = LiftingMotorPositionLimit[i] - DOWN_SETPOINT;
					LiftingMotorPositionSetpoint[i] = LiftingMotorBias[i] + FLASH_MEM[1];
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID() == 70){
		    LiftingMotorPositionSetpoint[0] = LiftingMotorBias[0] + FLASH_MEM[0];
            LiftingMotorPositionSetpoint[1] = LiftingMotorBias[1] + FLASH_MEM[0];
            LiftingMotorPositionSetpoint[2] = LiftingMotorBias[2];
            LiftingMotorPositionSetpoint[3] = LiftingMotorBias[3];    
		}
		else if(getID() == 71) {
			DANCING_MODE_FLAG = 0;
			if(getPositionSetpoint() == 1)
			{
				//load DANCING_MODE_RAISING_HEIGHT
				Set_CM_Speed(CAN2, 0, 0, 0, 0);
				BSP_DWT_DelayMs(1000);
				//delay 1s
				FLASH_MEM[0] = (u32)(CM2Encoder.ecd_angle - LiftingMotorBias[1]);
				writeFlash(FLASH_MEM, 1);
				HAS_RECEIVED_LOAD = 1;
			}
			
      for(u8 i = 0; i < 4; i++)
         LiftingMotorPositionSetpoint[i] = LiftingMotorBias[i] + DOWN_SETPOINT;
		}
		
		else if(getID()==0xFF){
			GO_ON_STAGE_ONE_KEY=true;
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==0xFE){
			GO_DOWN_STAGE_ONE_KEY=true;
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==0xFD){
			BREAK=true;
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==0xFC){
			ONE_KEY_DOWN_FRONT=true;
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==0xFB){
			ONE_KEY_DOWN_BACK=true;
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==0xFA){
			ONE_KEY_UP_FRONT=true;
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==0xF9){
			ONE_KEY_UP_BACK=true;
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID() == 63){
			//dancing mode begins
			INIT_FLAG = 0;
			DANCING_MODE_FLAG = 1;
		    LeftFrontReachUpper = 1;
            LeftBackReachUpper = 1;
            RightBackReachUpper = 1;
            RightFrontReachUpper = 1;  
			broken_time=receive_time=get_ms_ticks();			
		}
		else if(getID() == 64){
			DANCING_MODE_FLAG = 0;
			for(u8 i = 0; i < 4; i++)
				//LiftingMotorPositionSetpoint[i] = LiftingMotorBias[i] + UP_SETPOINT;
				LiftingMotorPositionSetpoint[i] = LiftingMotorBias[i] + FLASH_MEM[1];
			//dancing mode ends
			//need to set all LiftingMotors to raise up to the UP_SETPOINT
			//turn off friction wheel
            if(getPositionSetpoint() == 0)
			    FRICTION_WHEEL_STATE=false;
            else {
							FRICTION_WHEEL_STATE = true;
							for(u8 i = 0; i < 4; i++)
								//LiftingMotorPositionSetpoint[i] = LiftingMotorPositionLimit[i] - DOWN_SETPOINT;
								LiftingMotorPositionSetpoint[i] = LiftingMotorBias[i] + FLASH_MEM[1];
						}
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==0x00){
			//front wheels expanding acoording to the desired position offset sent by master
			LiftingMotorPositionSetpoint[0]=LiftingMotorBias[0]+8*getPositionSetpoint();
			LiftingMotorPositionSetpoint[1]=LiftingMotorBias[1]+8*getPositionSetpoint();
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==0x01){
			//front wheels withdrawing according to the desired position offset specified by master
			LiftingMotorPositionSetpoint[0]=LiftingMotorBias[0]+8*getPositionSetpoint();
			LiftingMotorPositionSetpoint[1]=LiftingMotorBias[1]+8*getPositionSetpoint();
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==0x02){
			//back wheels expanding according to the desired position offset sent by master
			LiftingMotorPositionSetpoint[2]=LiftingMotorBias[2]+8*getPositionSetpoint();
			LiftingMotorPositionSetpoint[3]=LiftingMotorBias[3]+8*getPositionSetpoint();
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==0x03){
			//back wheels withdrawing sccording to the desired position offset sent by master
			LiftingMotorPositionSetpoint[2]=LiftingMotorBias[2]+8*getPositionSetpoint();
			LiftingMotorPositionSetpoint[3]=LiftingMotorBias[3]+8*getPositionSetpoint();
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID() == 19){
			int16_t key_bit = getPositionSetpoint();
			u16 step = 200;
			if(key_bit < 30 && key_bit != 0){
				step = 200;
			if((key_bit>>0) & 1){														
				//Ctrl F
				//LiftingMotor[0] draws back
				modifyingBias(0, step);
			}
			if((key_bit>>1) & 1){
				//Ctrl G
				//LiftingMotor[1] draws back
				modifyingBias(1, step);
			}
			if((key_bit>>2) & 1){
				//Ctrl C
				//LiftingMotor[3] draws back
				modifyingBias(3, step);
			}
			if((key_bit>>3) & 1){
				//Ctrl V
				//LiftingMotor[2] draws back
				modifyingBias(2, step);
			}
							broken_time=receive_time=get_ms_ticks();
			}
			else if(key_bit ==35 ){

					modifyingBias(0, step);
					modifyingBias(1, step);
					broken_time=receive_time=get_ms_ticks();
			}
			else if(key_bit ==34){
					modifyingBias(2, step);
					modifyingBias(3, step);
					broken_time=receive_time=get_ms_ticks();
				
			}
		}
		else if(getID() == 0x14){
			int16_t key_bit = getPositionSetpoint();
			u16 step = 200;
			if(key_bit < 30 && key_bit != 0){
				step = 200;
			if((key_bit>>0) & 1){
				//Ctrl Shift F
				//LiftingMotor[0] expands
				modifyingUpperLimit(0, step);
			}
			if((key_bit>>1) & 1){
				//Ctrl Shift G
				//LiftingMotor[1] expands
				modifyingUpperLimit(1, step);
			}
			if((key_bit>>2) & 1){
				//Ctrl Shift C
				//LiftingMotor[3] expands
				modifyingUpperLimit(3, step);
			}
			if((key_bit>>3) & 1){
				//Ctrl Shift V
				//LiftingMotor[2] expands
				modifyingUpperLimit(2, step);
			}
							broken_time=receive_time=get_ms_ticks();
			}
			else if(key_bit ==35){

					modifyingUpperLimit(0, step);
					modifyingUpperLimit(1, step);
					broken_time=receive_time=get_ms_ticks();
			}
			else if(key_bit == 34){
					modifyingUpperLimit(2, step);
					modifyingUpperLimit(3, step);
					broken_time=receive_time=get_ms_ticks();
			}
			
		}
		else if(getID()==26){
			FRICTION_WHEEL_STATE=false;
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==27){
			FRICTION_WHEEL_STATE=false;
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==28){
			FRICTION_WHEEL_STATE=true;
			broken_time=receive_time=get_ms_ticks();
		}
		else if(getID()==0x55) {
			broken_time=receive_time=get_ms_ticks();
		}
		else broken_time=get_ms_ticks();
		
	
		
		
		//????DMA
    DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
    
		while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
    DMA_SetCurrDataCounter(DMA1_Stream1, BilateralCommunication_buffer);
    DMA_Cmd(DMA1_Stream1, ENABLE);

    
    
}
