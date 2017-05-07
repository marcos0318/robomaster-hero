#include "main.h"
#include "slave_communication.h"
#include "chasis_control.h"
#include "gimbal_control.h"

u32 ticks_msimg = (u32) - 1;

enum State{StaticState, MovingState};
bool SetpointStatic = false;
enum State GimbalState; 	
extern enum modeControl HERO;



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
			if(ticks_msimg % 20 == 0){
				DBUSBrokenLine = checkBrokenLine(ticks_msimg, DBUSBrokenLineCounter);
				//DBUSBrokenLine = 0;
				
				CAN1BrokenLine = checkBrokenLine(ticks_msimg, Wheel1BrokenLineCounter)
											|| checkBrokenLine(ticks_msimg, Wheel2BrokenLineCounter)
											|| checkBrokenLine(ticks_msimg, Wheel3BrokenLineCounter)
											|| checkBrokenLine(ticks_msimg, Wheel4BrokenLineCounter);
				CAN2BrokenLine = checkBrokenLine(ticks_msimg, YawBrokenLineCounter)
											|| checkBrokenLine(ticks_msimg, PitchBrokenLineCounter)
											|| checkBrokenLine(ticks_msimg, GunBrokenLineCounter)
											|| checkBrokenLine(ticks_msimg, CameraBrokenLineCounter);
				if(CAN1BrokenLine_prev == 1 && CAN1BrokenLine == 0){
					//setpoint_angle = 0;
					//GimbalFlag=2;
					direction = - output_angle*upperTotal/3600;
					GimbalFlag=3;
				}
				//CAN1BrokenLine = checkBrokenLine(ticks_msimg, CAN1BrokenLineCounter);
				//CAN2BrokenLine = checkBrokenLine(ticks_msimg, CAN2BrokenLineCounter);
				CAN1BrokenLine_prev = CAN1BrokenLine;
				CAN2BrokenLine_prev = CAN2BrokenLine;
			}
			if (DBUSBrokenLine == 0){
				//Gimbal Flag update
				if (CAN1BrokenLine == 0){
					if (DBUS_ReceiveData.rc.switch_right == 1) {
						GimbalFlag = 1;
					}
					else if (DBUS_ReceiveData.rc.switch_right == 3) {
						GimbalFlag = 2;
					}
					else if (DBUS_ReceiveData.rc.switch_right == 2) {
						GimbalFlag = 3;
					}
				}
				
			if (DBUS_ReceiveData.rc.switch_left == 1 || DBUS_ReceiveData.rc.switch_left == 3){ 

				/*******************************************************
				******************* DBUS Data Analyze ******************
				*******************************************************/

				//Analyse the data received from DBUS and transfer moving command					
				if(CAN2BrokenLine == 0){

					if(GimbalFlag == 1 || CAN1BrokenLine == 1){
						ChasisFlag = 2;
						FRIC_SET_THRUST_L(0);
						FRIC_SET_THRUST_R(0);
						keyboard_mouse_control();
					}
					DBUS_data_analysis();
		
					turning_speed_limit_control(ticks_msimg);
				  						 
					Set_CM_Speed(CAN2, wheel_outputs[0], wheel_outputs[1], wheel_outputs[2], wheel_outputs[3]);	
				}
				else
				{
					//chassis broken line
					//gimbal working
					//chassis state=4
					ChasisFlag = 4;
					for (int i=0; i<4; i++)
						PIDClearError(&states[i]);
					PIDClearError(&state_angle);
					Set_CM_Speed(CAN2, 0, 0, 0, 0);	
				}
		
				if(ticks_msimg % 20 == 0){
						state_control();
						
						
							
					
					tft_clear();
					/*for(uint8_t i=0; i<4; i++)
						tft_prints(1, i+2, "%d %d", i,wheel_feedbacks[i]);
					*/
					//tft_prints(1, 6, "yback=%.1f", gimbalPositionFeedback);
					//tft_prints(1, 7, "pback=%.1f", pitchPositionFeedback);
					//tft_prints(1, 8, "cback-%.1f", cameraPositionFeedback);
					//tft_prints(1, 7, "gyro:%d", output_angle);
					/*
					tft_prints(1, 5, "camSpS: %d", cameraSpeedSetpoint);
					tft_prints(1, 6, "camSpf:%d", GMCameraEncoder.filter_rate);
					tft_prints(1, 7, "camPst:%.1f", cameraPositionSetpoint);
					tft_prints(1, 8, "camPsf:%.1f", GMCameraEncoder.ecd_angle);
					*/
					tft_prints(1, 9, "state:%d", (int)HERO);
					tft_prints(1,2, "ticks:%d", ticks_msimg);
					tft_prints(1,3, "DBUS:%d %d", DBUSBrokenLineCounter, DBUSBrokenLine);
					tft_prints(1,4, "CAN1:%d %d", CAN1BrokenLineCounter, CAN1BrokenLine);
					tft_prints(1,5, "CAN2:%d %d", CAN2BrokenLineCounter, CAN2BrokenLine);
					//tft_prints(1,2, "dir:%d", direction);
					//tft_prints(1,3, "gyro:%d", output_angle);
					//tft_prints(1,4, "chsAgl: %d", setpoint_angle);
					//tft_prints(1,5, "yawSp: %.1f", gimbalPositionSetpoint);
//					tft_prints(1,6, "chF:%d", ChasisFlag);
//					tft_prints(1,7, "qe_tn:%d", is_qe_turning);
//					tft_prints(1,8,	"Gflag:%d", GimbalFlag);
					
					tft_update();
					
				}
			

				if(DBUS_ReceiveData.rc.switch_left == 2){
					Set_CM_Speed(CAN1,0,0,0,0);
					Set_CM_Speed(CAN2,0,0,0,0);
				}
			}
				//if ( ticks_msimg % 20 == 0 ){
					//tft_clear();
					

					//tft_update();
				//}
			}
			else		
			{
				Set_CM_Speed(CAN1,0,0,0,0);
				Set_CM_Speed(CAN2,0,0,0,0);
			}				
		}
	}
} 
