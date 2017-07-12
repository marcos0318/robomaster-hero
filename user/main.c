#include "main.h"
#include "slave_communication.h"
#include "chasis_control.h"
#include "gimbal_control.h"

u32 ticks_msimg = (u32) - 1;

enum State{StaticState, MovingState};
bool SetpointStatic = false;
enum State GimbalState; 	
extern enum modeControl HERO;

extern volatile uint32_t foo;
extern volatile float bar;

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
	//ENCODER_Init();
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
		
	
	//incPIDinit(&gimbalSpeedMoveState);
	//incPIDinit(&pitchSpeedMoveState);

	//incPIDset(&gimbalSpeedMoveState, 70, 3.7, 0);
	//incPIDset(&pitchSpeedMoveState, 70, 3.7, 0);

	mouse_prev = DBUS_ReceiveData.mouse.xtotal;

	while (1){	

		if (ticks_msimg != get_ms_ticks()){
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			
			
			if (ticks_msimg % 20 == 0) {
				shootingWheelSpeed = convertToShootingSpeed( InfantryJudge.RealVoltage );
				
				if (shootingWheelSpeed > 2000 ) {
					shootingWheelSpeed = 2000;
				}
				
				if (shootingWheelSpeed < 0) {
						shootingWheelSpeed = 0;
				}
			}
			
			
			DBUS_data_analysis();	
			//GUN_SetMotion();
			//check 
			if(ticks_msimg % 20 == 0){
				DBUSBrokenLine = checkBrokenLine(ticks_msimg, DBUSBrokenLineCounter);
				
				CAN2BrokenLine = checkBrokenLine(ticks_msimg, Wheel1BrokenLineCounter)
											|| checkBrokenLine(ticks_msimg, Wheel2BrokenLineCounter)
											|| checkBrokenLine(ticks_msimg, Wheel3BrokenLineCounter)
											|| checkBrokenLine(ticks_msimg, Wheel4BrokenLineCounter);
				CAN1BrokenLine = checkBrokenLine(ticks_msimg, YawBrokenLineCounter)
											|| checkBrokenLine(ticks_msimg, PitchBrokenLineCounter)
											|| checkBrokenLine(ticks_msimg, GunBrokenLineCounter);
											//|| checkBrokenLine(ticks_msimg, CameraBrokenLineCounter);
				//TODO: move to somewhere else
				
				if(CAN1BrokenLine_prev == 1 && CAN1BrokenLine == 0){
					CAN1BrokenLineRecover = 1;
					direction = -output_angle*upperTotal/3600;
					gimbalPositionSetpoint = bufferedGimbalPositionSetpoint = 0;					//gimbal go back to the middle
					setpoint_angle = output_angle;		//chassis stay current angle
					DBUS_ReceiveData.mouse.xtotal=0;
					xtotal=pre_xtotal=0;
				}
				else {
					CAN1BrokenLineRecover = 0;
				}
				if(CAN2BrokenLine_prev == 1 && CAN2BrokenLine == 0){
					CAN2BrokenLineRecover = 1;
												//may need to reinitialize CAN2 so as to send SPEED message
					setpoint_angle = output_angle;		//chassis stay current angle
				}
				else {
					CAN2BrokenLineRecover = 0;
				}
				
				if(CAN2BrokenLine_prev == 0 &&CAN2BrokenLine == 1) {
				
					tft_clear_line(2);
					tft_prints(1,2,"CAN2NOTWORKING");
					for (int i=0; i<4; i++) {
						PIDClearError(&states[i]);
						wheel_setpoints[i] = 0;
						wheel_outputs[i]=0;
					}
					fPIDClearError(&state_angle);			
					Set_CM_Speed(CAN2,0,0,0,0);
				}
				if(DBUSBrokenLine_prev == 1 && DBUSBrokenLine == 0){
					DBUSBrokenLineRecover = 1;
				} else {
					DBUSBrokenLineRecover = 0;
				}

				
				CAN1BrokenLine_prev = CAN1BrokenLine;
				CAN2BrokenLine_prev = CAN2BrokenLine;
				DBUSBrokenLine_prev = DBUSBrokenLine;
			}


			if (DBUSBrokenLine == 0) {
				tft_clear_line(6);
				tft_prints(1,6,"DBUSWORKING");
				//DBUS online
				if (CAN1BrokenLine == 0){
					//Can1 online
					//Check the change of switch
					if(HERO == RUNNING_MODE && GimbalFlag != 1 && CAN2BrokenLine == 0)
						ChasisFlag = 1;
					if(HERO == RUNNING_MODE && !(DBUS_ReceiveData.rc.switch_left == 1 && DBUS_ReceiveData.rc.switch_right == 1))
						GimbalFlag = 3;
			
					if (LastDBUSLeftSwitch != DBUS_ReceiveData.rc.switch_left || LastDBUSRightSwitch != DBUS_ReceiveData.rc.switch_right){
					  
					  if (DBUS_ReceiveData.rc.switch_left == 1 ) {
					    //Left up mode
					    if (DBUS_ReceiveData.rc.switch_right == 1) {
				 	      GimbalFlag = 1;
								//Left switch and Right switch all up:  release all control
					    }
					    else if (DBUS_ReceiveData.rc.switch_right == 2) {
								GimbalFlag = 3;
								//Left up, Right down: controlled by keyboard, mouse (competition mode)
								//all need to go back to initial position
								//go to the state 0
								HERO = RUNNING_MODE;
								switch_and_send();
					    }
							else GimbalFlag = 3;
					  }
					  else GimbalFlag = 3;
					  if (DBUS_ReceiveData.rc.switch_right == 3 ) {
					    GimbalFlag = 3;
							//Right middle, control by Remote Controller
							//Left changed from middle to Up, similar to G, go to the next step
					    if (DBUS_ReceiveData.rc.switch_left == 1 && LastDBUSLeftSwitch == 3) {
					  	  //go to next state
							if(HERO!=DOWN_BACK_WHEEL && HERO != FRONT_WHEEL_DOWN){
								HERO+=1;
								switch_and_send();
							}
							else if(HERO == DOWN_BACK_WHEEL){
								HERO=RUNNING_MODE;
								ChasisFlag = 1;
								GimbalFlag = 3;
								direction = - output_angle*upperTotal/3600;
								filter_rate_limit = FOR_JOHN_MAX_RUNNING_SPEED;
								speed_multiplier = FOR_JOHN_MAX_RUNNING_SPEED;
								//withdraw lower pneumatic
								lower_pneumatic_state = false;
								pneumatic_control(1, 0);
								pneumatic_control(2, 0);
								pneumatic_control(3, 0);
								pneumatic_control(4, 0);
								LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
								DataMonitor_Send(5, 2);
							} else if(HERO == FRONT_WHEEL_DOWN) {
								HERO = INTO_RI_MODE;
								switch_and_send();
							}
								
								if(HERO == FRONT_WHEEL_UP || HERO == UPPER_HORIZONTAL_PNEUMATIC_EXTENDS || HERO != BACK_WHEEL_UP ||HERO != SPEED_LIMITATION)
								{								
									LOAD_FLASH = 0;
									RC_CTRL = 0;
									RC_CTRL_SHIFT = 0;
									step = 0;
									LeftJoystick = 1;							
								}
					    }
					    if (DBUS_ReceiveData.rc.switch_left == 2 && LastDBUSLeftSwitch == 3) {
								//go to prev state
							if(HERO == RUNNING_MODE || HERO == REVERSE_RUNNING_MODE) {}
							else if(HERO == BACK_WHEEL_DOWN) {
								HERO = REVERSE_RUNNING_MODE;
								pneumatic_control(1, 0);
								DataMonitor_Send(5, 3);
							}
							else if(HERO == FRONT_WHEEL_DOWN) {
								HERO = BACK_WHEEL_DOWN;
								switch_and_send();
							}
							else if(HERO != VERTICAL_PNEUMATIC_WITHDRAWS)
								backState[HERO--]();
							else {
								backState[HERO]();
								HERO = UPPER_HORIZONTAL_PNEUMATIC_EXTENDS;
					    	}
						}
					}
					if(DBUS_ReceiveData.rc.switch_left == 3 && DBUS_ReceiveData.rc.switch_right == 3 && (HERO == BACK_WHEEL_UP || HERO == SPEED_LIMITATION)){
						//Left Middle, Right Middle						DONE
						//can only manipulate LiftingMotor by RC under BACK_WHEEL_UP and SPEED_LIMITATION mode				DONE
						
						LeftJoystick = 0;
						if(DBUS_ReceiveData.rc.ch3 > 600 || DBUS_ReceiveData.rc.ch3 < -600 || DBUS_ReceiveData.rc.ch2 > 600 || DBUS_ReceiveData.rc.ch2 < -600)
						{
							LOAD_FLASH = 1;
							//after switch_and_send()
							//need to turn off LOAD_FLASH
							
							
							if(DBUS_ReceiveData.rc.ch3 > 600 || DBUS_ReceiveData.rc.ch3 < -600){
								//ch3, vertically move, control LeftBack (id 3) and RightBack (id 2)
								if(DBUS_ReceiveData.rc.ch3 > 600){
								//up, LiftingMotors go up, using CTRL+SHIFT
									RC_CTRL_SHIFT = 1;
									RC_CTRL = 0;						
									step =34;
								} else if(DBUS_ReceiveData.rc.ch3 < -600){
								//down, LiftingMotors go down, using CTRL
									RC_CTRL = 1;
									RC_CTRL_SHIFT = 0;
									step =34;
								}
								else step = 0;
								//modify step to be even
							}
							else{
							//ch2, horizontally move, control LeftFront (id 0) and RightFront (id 1)					
								if(DBUS_ReceiveData.rc.ch2 > 600){
								//Left, LiftingMotors go up, using CTRL+SHIFT
									RC_CTRL_SHIFT = 1;
									RC_CTRL = 0;
									step = 35;
								} else if(DBUS_ReceiveData.rc.ch2 < -600){
								//Right, LiftingMotors go down, using CTRL
									RC_CTRL = 1;
									RC_CTRL_SHIFT = 0;
									step =35;
								}
								else step = 0;
							//modify step to be odd
							}
						
						//turn off left joystick control for gimbal
						//pitch DONE
						//yaw
						//in SPEED_LIMITATION mode, detect whether to load DANGIN_MODE_RASING_HEIGHT
						}
						else {
							RC_CTRL = RC_CTRL_SHIFT = 0;
							step = 0;
						}
					}
					else {
						//turn on left joystick control for gimbal
						LeftJoystick = 1;
						step = 0;
					}

					if(LastDBUSRightSwitch != DBUS_ReceiveData.rc.switch_right) {
						if(DBUS_ReceiveData.rc.switch_right == 3 ) {
							HERO = RUNNING_MODE;
							switch_and_send();
						}
						else if(DBUS_ReceiveData.rc.switch_right == 2) {
							HERO = REVERSE_RUNNING_MODE;
							switch_and_send();
						}
					}
				} 
				}
				else {
					GimbalFlag = 1;
				}


				if(CAN2BrokenLine == 0){
					if(HERO == RUNNING_MODE && GimbalFlag != 1 && !FOR_JOHN_SHIFT_G_SPECIAL_MODE)
						ChasisFlag = 1;
					
					tft_clear_line(2);
					tft_prints(1,2,"CAN2WORKING");
					turning_speed_limit_control(ticks_msimg);
					Set_CM_Speed(CAN2, wheel_outputs[0], wheel_outputs[1], wheel_outputs[2], wheel_outputs[3]);	
					tft_clear_line(7);
					tft_prints(1,7,"CAN1 2:%d %d",transmit_return_value_CAN1, transmit_return_value_CAN2);
				}
				else
				{
					//chassis broken line
					//gimbal working
					//chassis state=4
					//tft_clear_line(2);
					tft_clear_line(2);
					tft_prints(1,2,"CAN2NOTWORKING");
					for (int i=0; i<4; i++) {
						PIDClearError(&states[i]);
						wheel_setpoints[i] = 0;
						wheel_outputs[i]=0;
					}
					fPIDClearError(&state_angle);
					//Set_CM_Speed(CAN2, 0, 0, 0, 0);	
				}

				//if(TIM_7_Counter % 20 == 0)
					//state_control();
			}
			else {
				//Dbus Offline
				tft_clear_line(6);
				tft_prints(1,6,"DBUSNOTWORKING");
				for (int i=0; i<4; i++) {
					PIDClearError(&states[i]);
					wheel_setpoints[i] = 0;
					wheel_outputs[i]=0;
				}
				fPIDClearError(&state_angle);
				Set_CM_Speed(CAN2, 0, 0, 0, 0);	
				
			}
			if(TIM_7_Counter % 20 == 0)
					state_control();
			LastDBUSLeftSwitch = DBUS_ReceiveData.rc.switch_left;
			LastDBUSRightSwitch = DBUS_ReceiveData.rc.switch_right;		
			
			
			
			if(ticks_msimg%20==0){
				//tft_clear();
				tft_clear_line(9);
				tft_prints(1, 9, "state:%d", (int)HERO);
				tft_prints(1,2, "ticks:%d", ticks_msimg);
				tft_clear_line(3);
				tft_clear_line(4);
				tft_clear_line(5);
				tft_prints(1,3, "DBUS:%d %d %d", DBUSBrokenLineCounter, DBUSBrokenLine, DBUSBrokenLineRecover);
				tft_prints(1,4, "CAN1:%d %d %d", CAN1BrokenLineCounter, CAN1BrokenLine, CAN1BrokenLineRecover);
				tft_prints(1,5, "CAN2:%d %d %d", CAN2BrokenLineCounter, CAN2BrokenLine, CAN2BrokenLineRecover);
		
				//tft_prints(1,6,"dir:%d    Gf:%d", direction, GimbalFlag);
				
//				tft_clear_line(3);
//				tft_clear_line(4);
//				tft_clear_line(5);
				tft_clear_line(6);
				//tft_prints(1, 6, "LF:%d", LOAD_FLASH);
				tft_prints(1, 6, "LF_fr:%d", CM1Encoder.filter_rate);
				tft_clear_line(7);
				tft_clear_line(8);
				tft_prints(1,7,"LF:%d RF:%d",checkBrokenLine(ticks_msimg, Wheel1BrokenLineCounter) ,checkBrokenLine(ticks_msimg, Wheel2BrokenLineCounter) );
				tft_prints(1,8,"LB:%d RB:%d",checkBrokenLine(ticks_msimg, Wheel4BrokenLineCounter) ,checkBrokenLine(ticks_msimg, Wheel3BrokenLineCounter));
//				tft_clear_line(10);
				//tft_prints(1,7,"spA:%d    Cf:%d", setpoint_angle, ChasisFlag);
				//tft_prints(1,8,"gyro:%dfr:%d", output_angle, wheel_feedbacks[0] );
//				tft_prints(1,3,"psp:%f", pitchPositionSetpoint);
//				tft_prints(1,4,"pfd:%f", pitchPositionFeedback);
//				tft_prints(1,5,"ssp:%f", pitchSpeedSetpoint);
//				tft_prints(1,6,"sfd:%f",pitchSpeedFeedback );
				//tft_prints(1,10,"wout:%d",wheel_outputs[0]);	
				tft_clear_line(10);
				tft_prints(1, 10, "shooSpd: %d", shootingWheelSpeed);
				//tft_prints(1,10,"ss:%d d:%d",FOR_JOHN_SHIFT_G_SPECIAL_MODE, state_delay);
			  
				//tft_prints(1,11,":%.1f", gimbalSpeedSetpoint);
				//tft_prints(1,11,":%d", chasis_turning_speed);
				tft_clear_line(11);
				tft_prints(1, 11, "V S: %.1f %.1f", InfantryJudge.RealVoltage, InfantryJudge.LastShotSpeed );
				tft_update();
			}	
			
	
			
		}
	}
} 
