#include "main.h"
#include "function_list.h"
#include "Lifting_Motor_Control.h"
#include "GoOnStage.h"
#include "initialization_process.h"

static u32 ticks_msimg = (u32)-1;

void init(){
//	InfantryJudge.LastBlood = 1500;
	SysTick_Init();  
	//Dbus_init();//usart1
	//buzzer_init();	 //initialization of buzzer
	tft_init(2,WHITE,BLACK,BLACK);
	LED_master_init();
	DataMonitor_Init();
	gyro_init();
	//ADC1_init();
	//judging_system_init(); //usart3
	Bilateral_Init();
	gyro_init();
	//pneumatic_init();
	CAN1_Configuration();
	CAN2_Configuration();
	gyro_init();
	gyro_cal();	
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	LiftingMotorInit();
	Limit_Switch_init();
	BSP_DWT_InitConfig();
	TIM7_Int_Init(83,999);
}









//PID controls

//The control of filter rate of wheels 
// Structure to strore PID data 
u32 flash0;
u32 flash1;

int main(void)
{	
	init();
//	DBUS_ReceiveData.mouse.ytotal=0;
	flash0 = readFlash(0);
	flash1 = readFlash(1);
	
	
	
	while (1)  
	{	


		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			LED_blink(LED1);
			readFeedback();
				
			if(ticks_msimg % 500 ==0) {
				//tft_clear();
				tft_clear_line(2);
				tft_prints(1,2,"BC:%d C2:%d", BROKEN_CABLE, CAN2BrokenLine);
				//tft_prints(1,4,"btime %d", broken_time);
				//tft_prints(1,5,"rtime %d", receive_time);
				for(u8 i=3;i<10;i++) 
          tft_clear_line(i); 
				//for(uint8_t i=0;i<4;i++) 
          //tft_prints(1,i+6,"sp %d %d", i+1, LiftingMotorPositionSetpoint[i]);
				//tft_prints(1,3,"LF sp %d", LiftingMotorPositionSetpoint[0]);
				//tft_prints(1,4,"RB sp %d", LiftingMotorPositionSetpoint[2]);
				//tft_prints(1,5,"LF ecd %f", CM1Encoder.ecd_angle);
				//tft_prints(1,3,"BT:%d ",INIT_protection_timer_begin);
				//tft_prints(1,4,"RT:%d ",INIT_protection_timer_reach);
				tft_prints(1,3,"RFBIAS:%d", LiftingMotorBias[1]);
				tft_prints(1,4,"RFH:%f", (float)(CM2Encoder.ecd_angle - LiftingMotorBias[1]));
				tft_prints(1,5,"RBH:%f", (float)(CM3Encoder.ecd_angle - LiftingMotorBias[2]));
//				tft_prints(1,5,"DT:%d", INIT_protection_timer_down);
//				tft_prints(1,6,"RF fr %d", CM2Encoder.filter_rate);
//				tft_prints(1,7,"RF speed %d", LiftingMotorOutput[1]);
				tft_prints(1, 8, "LF%d RF%d ", gpio_read_input(LeftFront), gpio_read_input(RightFront));
				tft_prints(1, 9, "LB%d RB%d ", gpio_read_input(LeftBack), gpio_read_input(RightBack));
				//tft_prints(1, 9, "num_LF: %d", num_of_touch(LeftFront));
				//tft_prints(1,6, "f0:%d f1:%d", flash0, flash1);
				tft_prints(1,6, "US:%d", FLASH_MEM[0]);
				tft_prints(1,7,"F0:%d",readFlash(0));
				//tft_prints(1,8, "DM:%d", FLASH_MEM[1]);
				//tft_prints(1,9, "F1:%d", readFlash(1));
        //for (int i=0;i<4;i++) 
          //tft_prints(1,i+2,"Bias%d %d",i+1, LiftingMotorBias[i]); 
				u8 temp1 = getID();
				tft_clear_line(10);
				tft_prints(1,10,"L %d",HAS_RECEIVED_LOAD);
				//tft_prints(1,10, "ID:%d", temp);
				//tft_prints(1,10, "R_F:%d D_F: %d", HAS_ALL_REACHED_FLAG, HAS_ALL_DOWN_FLAG);
				tft_clear_line(11);
				//tft_prints(1,11,"BT:%d RT:%d DT:d",INIT_protection_timer_begin, INIT_protection_timer_reach, INIT_protection_timer_down);
				//tft_prints(1, 11, "UBF:%d DBF:%d", INIT_protection_up_begin_flag, INIT_protection_down_begin_flag);
				u8 temp2 = getPositionSetpoint();
				tft_prints(1,11,"ID: %d kb:%d", temp1, temp2);
				tft_update();
			}
	
			
			
		}//main loop with ticks	
	}
	
	
}	//main


