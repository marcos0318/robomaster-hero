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
	ADC1_init();
	//judging_system_init(); //usart3
	Bilateral_Init();
	gyro_init();
	pneumatic_init();
	CAN1_Configuration();
	CAN2_Configuration();
	gyro_init();
	gyro_cal();
	//Friction_wheel_init();
	GUN_Init();
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	LiftingMotorInit();
	Limit_Switch_init();
	TIM7_Int_Init(83,999);
}









//PID controls

//The control of filter rate of wheels 
// Structure to strore PID data 

int main(void)
{	
	init();
//	DBUS_ReceiveData.mouse.ytotal=0;
	
	
	
	
	while (1)  
	{	


		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			LED_blink(LED1);
			readFeedback();
			
			
			
			
			if(ticks_msimg>10000){
				if(!FRICTION_WHEEL_STATE)
					friction_wheel_setpoint=0;
				else {
					if(friction_wheel_setpoint<300)
						friction_wheel_setpoint+=1;
				}
				FRIC_SET_THRUST_L(friction_wheel_setpoint);
				FRIC_SET_THRUST_R(friction_wheel_setpoint);
			}
			if(ticks_msimg % 50 ==0) {
				//tft_clear();
				//tft_prints(1,3,"Broken %d", BROKEN_CABLE);
				//tft_prints(1,4,"btime %d", broken_time);
				//tft_prints(1,5,"rtime %d", receive_time);
				for(uint8_t i=0;i<4;i++) 
          tft_clear_line(i+6); 
        for (int i=0;i<4;i++) 
          tft_clear_line(i+2);
				//for(uint8_t i=0;i<4;i++) 
          //tft_prints(1,i+6,"sp %d %d", i+1, LiftingMotorPositionSetpoint[i]);
				tft_prints(1,3,"LF sp %d", LiftingMotorPositionSetpoint[0]);
				tft_prints(1,4,"LF Bias %d", LiftingMotorBias[0]);
				tft_prints(1,5,"LF limit %d", LiftingMotorPositionLimit[0]);
				tft_prints(1,6,"LF ecd %f", CM1Encoder.ecd_angle);
				tft_prints(1,7,"LF speed %d", LiftingMotorOutput[0]);
				tft_prints(1, 8, "GPIO_LF: %d", gpio_read_input(LeftFront));
				tft_prints(1, 9, "num_LF: %d", num_of_touch(LeftFront));
        //for (int i=0;i<4;i++) 
          //tft_prints(1,i+2,"Bias%d %d",i+1, LiftingMotorBias[i]); 
				uint8_t temp = getID();
				tft_clear_line(10);
				
				tft_prints(1,10,"ID:%d",temp);
				tft_clear_line(11);
				tft_prints(1,11,"key:%d",getPositionSetpoint());
				tft_update();
			}
	
			
			
		}//main loop with ticks	
	}
	
	
}	//main

