#include "main.h"
#include "slave_communication.h"
#include "chasis_control.h"
#include "gimbal_control.h"

u32 ticks_msimg = (u32) - 1;

enum State{StaticState, MovingState};
bool SetpointStatic = false;
enum State GimbalState; 	

void init(){

	SysTick_Init();  
	Dbus_init();//usart1
	//buzzer_init();	 //initialization of buzzer
	tft_init(2, WHITE, BLACK, BLACK);
	LED_master_init();
	gyro_init();
	ADC1_init();
	judging_system_init(); //usart3
	gyro_init();
	pneumatic_init();
	CAN1_Configuration();
	CAN2_Configuration();
	gyro_init();
	gyro_cal();
	TIM5_Int_Init(24, 13124);// 256hz //3.9xx ms for gyro usage
	DataMonitor_Init();
	ENCODER_Init();
	GUN_Init();
  TIM7_Int_Init(83,999);
}


//int32_t inverse = 1;//if inv == 1, move forward, else if inv == -1, move backward

//The coorperation of gimbal and the chasis
//The direction is from 0 to 8192
//The gyro of chasis is ranged from 0 to 3600, so we need conversion


int main(void)
{	
	init();
	DBUS_ReceiveData.mouse.ytotal = 0;
	
	//init the buffers with zero 
	for (int i=0; i<4; i++)
		for (int j=0; j<BUFFER_LENGTH; j++)
			buffer[i][j] = 0;
		
	
	incPIDinit(&gimbalSpeedMoveState);
	incPIDinit(&pitchSpeedMoveState);

	incPIDset(&gimbalSpeedMoveState, 70, 3.7, 0);
	incPIDset(&pitchSpeedMoveState, 70, 3.7, 0);

	mouse_prev = DBUS_ReceiveData.mouse.xtotal;

	while (1){	

		if (ticks_msimg != get_ms_ticks()){
			ticks_msimg = get_ms_ticks();  //maximum 1000000	

			//filter_rate limit control
			

			if (DBUS_ReceiveData.rc.switch_left == 1 || DBUS_ReceiveData.rc.switch_left == 3){ 

				/*******************************************************
				******************* DBUS Data Analyze ******************
				*******************************************************/

				//Analyse the data received from DBUS and transfer moving command					
				
				DBUS_data_analysis();
				//direction = -DBUS_ReceiveData.rc.ch2*2 + -DBUS_ReceiveData.mouse.xtotal*4 ;
				/*
				if(DBUS_ReceiveData.rc.switch_left == 1) {		//auto follow mode
					direction += (-DBUS_ReceiveData.rc.ch2/300 + -DBUS_ReceiveData.mouse.x);
					setpoint_angle = -direction * 3600/upperTotal;
					gimbalPositionSetpoint = direction + output_angle*upperTotal/3600;
					if (gimbalPositionSetpoint > 1500) gimbalPositionSetpoint = 1500;
					if (gimbalPositionSetpoint < -1500) gimbalPositionSetpoint = -1500;
				}
				*/
				//keyboard-mouse mode, chasis will turn if mouse go beyong the boundary				

				

				/*******************************************************
				*********** Chasis turing speed limit control **********
				*******************************************************/				
				
				
				/*******************************************************
				******************** Power Control *********************
				*******************************************************/
				
				turning_speed_limit_control(ticks_msimg);
				/*******************************************************
				******** Chasis turing speed limit control ends ********
				*******************************************************/

				
				//position setpoint is done above
									


				/*******************************************************
				*************** Pitch setpoint control *****************
				*******************************************************/


				/*******************************************************
				************ Yaw & Pitch velocity control **************
				*******************************************************/			
				
				
				
				
				
				
				//mock speed here
				//gimbalSpeedSetpoint = DBUS_ReceiveData.rc.ch2 * 0.5;

								
				

				/*******************************************************
				******************* Camera control *********************
				*******************************************************/	
								//call the acturater function
				//if (ticks_msimg % 20 == 0)
				

		  						 
				Set_CM_Speed(CAN2, wheel_outputs[0], wheel_outputs[1], wheel_outputs[2], wheel_outputs[3]);	
			
			/*
			else if (DBUS_ReceiveData.rc.switch_left == 3) { //Also the stop mode now
				Set_CM_Speed(CAN1,0,0,0,0);
				Set_CM_Speed(CAN2,0,0,0,0);
			} 
			*/
		
				if(ticks_msimg % 20 == 0){
						state_control();
					
					
				
					
										
					//for(uint8_t i=2; i<12; i++)
					//	tft_clear_line(i);
					//for(uint8_t i=0; i<4; i++){
					//	tft_prints(1, i+2, "LMP %d %d", i, LiftingMotorSetpoint[i]);
					//}
					
					tft_clear();
					for(uint8_t i=0; i<4; i++)
						tft_prints(1, i+2, "%d %d", i,wheel_feedbacks[i]);

					//tft_prints(1, 6, "yback=%.1f", gimbalPositionFeedback);
					//tft_prints(1, 7, "pback=%.1f", pitchPositionFeedback);
					//tft_prints(1, 8, "cback-%.1f", cameraPositionFeedback);
					//tft_prints(1, 7, "gyro:%d", output_angle);

					tft_prints(1, 5, "camSpS: %d", cameraSpeedSetpoint);
					tft_prints(1, 6, "camSpf:%d", GMCameraEncoder.filter_rate);
					tft_prints(1, 7, "camPst:%.1f", cameraPositionSetpoint);
					tft_prints(1, 8, "camPsf:%.1f", GMCameraEncoder.ecd_angle);

					tft_update();
					
				}
				
				//Set_CM_Speed(CAN1, 0, 0, 0, 0);
				//Set_CM_Speed(CAN2, 0, 0, 0, 0);
			//}		

			if(DBUS_ReceiveData.rc.switch_left == 2){
				Set_CM_Speed(CAN1,0,0,0,0);
				Set_CM_Speed(CAN2,0,0,0,0);
			}
			
			if ( ticks_msimg % 20 == 0 ){
				//tft_prints(1,9,"mode:%d", DBUS_ReceiveData.rc.switch_left );
				//tft_update();
			}

			
			
		} //main loop with ticks	
	
	}
	}
} //main
