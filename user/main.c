#include "main.h"
#include "function_list.h"



#define POWER_BUFFER_LENGTH 20
#define ANGLE_PID_LIMIT 500
#define MOVING_BOUND_1 200
#define MOVING_BOUND_2 450
#define SPEED_SETPOINT_LIMIT 800
#define UP_SETPOINT 255000						//determined by the height of the pneumatic, where pneumatice can be put on the stage precisely
#define DOWN_SETPOINT 3000						//determined by the relative height between the pneumatic and the wheels, whe wheels should be put on the stage precisely
#define TOTALLY_DOWN_SETPOINT 1000
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
	Friction_wheel_init();
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
}







int32_t LiftingMotorSpeedFeedback[4];
float LiftingMotorPositionFeedback[4];

int32_t LiftingMotorSpeedSetpoint[4];
int32_t LiftingMotorSpeedSetpointBuffered[4];


volatile int32_t LiftingMotorPositionSetpoint[4];
int32_t LiftingMotorSpeedSetpointBuffered[4];

volatile bool ONE_KEY_UP_FRONT;
volatile bool ONE_KEY_UP_BACK;
volatile bool ONE_KEY_DOWN_FRONT;
volatile bool ONE_KEY_DOWN_BACK;
volatile bool GO_ON_STAGE_ONE_KEY=false;		//become false when the process is finished, or by having received the break command
volatile bool GO_DOWN_STAGE_ONE_KEY=false;		//become false when the process is finished, or by having received the break command
bool GO_ON_STAGE_ONE_KEY_PREV=false;
bool GO_DOWN_STAGE_ONE_KEY_PREV=false;
volatile bool BREAK=false;
u32 ticks_msimg_on_prev=0, ticks_msimg_down_prev=0;
struct fpid_control_states LiftingMotorPositionState[4] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
float LMpos_kp = 0.3;
float LMpos_ki = 0.0005;
float LMpos_kd = 20;

//PID controls

//The control of filter rate of wheels 
// Structure to strore PID data 
struct pid_control_states LiftingMotorState[4] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};

int32_t LiftingMotorOutput[4] = {0,0,0,0};
int32_t kp = 80, ki = 4, kd = 1;

int main(void)
{	
	init();
//	DBUS_ReceiveData.mouse.ytotal=0;
	
	
	
	
	while (1)  
	{	


		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			LiftingMotorSpeedFeedback[0] = CM1Encoder.filter_rate;
			LiftingMotorSpeedFeedback[1] = CM2Encoder.filter_rate;
			LiftingMotorSpeedFeedback[2] = CM3Encoder.filter_rate;
			LiftingMotorSpeedFeedback[3] = CM4Encoder.filter_rate;
			LiftingMotorPositionFeedback[0] = CM1Encoder.ecd_angle;
			LiftingMotorPositionFeedback[1] = CM2Encoder.ecd_angle;
			LiftingMotorPositionFeedback[2] = CM3Encoder.ecd_angle;
			LiftingMotorPositionFeedback[3] = CM4Encoder.ecd_angle;
			
			if(BREAK && GO_ON_STAGE_ONE_KEY) GO_ON_STAGE_ONE_KEY=false;
			if(BREAK && GO_DOWN_STAGE_ONE_KEY) GO_DOWN_STAGE_ONE_KEY=false;
			if(!GO_ON_STAGE_ONE_KEY_PREV && GO_ON_STAGE_ONE_KEY) {
				ticks_msimg_on_prev=ticks_msimg;			//starting counting time for the procedure
				for(uint8_t i=0;i<4;i++)
					LiftingMotorPositionSetpoint[i]=UP_SETPOINT;
				GO_ON_STAGE_ONE_KEY=false;
			}
			if(!GO_DOWN_STAGE_ONE_KEY_PREV && GO_DOWN_STAGE_ONE_KEY) {
				ticks_msimg_down_prev=ticks_msimg;			//starting counting time for the procedure
				for(uint8_t i=0;i<4;i++)
					LiftingMotorPositionSetpoint[i]=DOWN_SETPOINT;
				GO_DOWN_STAGE_ONE_KEY=false;
			}
			/*
			if(ticks_msimg%20==0){
					FRIC_SET_THRUST_L(700);
					FRIC_SET_THRUST_R(700);
			}
			*/
			/*
			*Go on stage:
			*Five procedures:
			*First: 	FRONT and BACK both UP
			*Second:  pneumatic valve
			*Third:		BACK DOWN											Notice we use the back side to go on stage
			*Fourth:	Chasis moves backward					Again we are using the backward to go on stage
			*Fifth:		FRONT DOWN
			*/
			
			
//			if(GO_ON_STAGE_ONE_KEY){
//				
//				if(ticks_msimg-ticks_msimg_on_prev == 50000){
//					//pneumatic control
//					
//				}
//				if(ticks_msimg-ticks_msimg_on_prev >= 100000){
//					//LiftingMotorPositionSetpoint[2]=LiftingMotorPositionSetpoint[3]=DOWN_SETPOINT;
//					GO_ON_STAGE_ONE_KEY=false;
//				}
//			}
						
			
			/*Go down stage:
			*Four procedures:
			*First: FRONT UP
			*Second: Chasis move
			*/
			
//			if(GO_DOWN_STAGE_ONE_KEY){
//				for(uint8_t i=0;i<4;i++)
//					LiftingMotorPositionSetpoint[i]=TOTALLY_DOWN_SETPOINT;
//				if(ticks_msimg-ticks_msimg_on_prev == 50000){
//					//pneumatic control
//					
//				}
//				if(ticks_msimg-ticks_msimg_on_prev >= 100000){
//					//LiftingMotorPositionSetpoint[2]=LiftingMotorPositionSetpoint[3]=DOWN_SETPOINT;
//					GO_DOWN_STAGE_ONE_KEY=false;
//				}
//			}
			
			
			
			
			
			
			if(ONE_KEY_UP_FRONT){LiftingMotorPositionSetpoint[0]=LiftingMotorPositionSetpoint[1]=UP_SETPOINT;}
			else if(ONE_KEY_DOWN_FRONT){LiftingMotorPositionSetpoint[0]=LiftingMotorPositionSetpoint[1]=DOWN_SETPOINT;}
			if(ONE_KEY_UP_BACK){LiftingMotorPositionSetpoint[2]=LiftingMotorPositionSetpoint[3]=UP_SETPOINT;}
			else if(ONE_KEY_DOWN_BACK){LiftingMotorPositionSetpoint[2]=LiftingMotorPositionSetpoint[3]=DOWN_SETPOINT;}
				

			GO_ON_STAGE_ONE_KEY_PREV=GO_ON_STAGE_ONE_KEY;			//don't forget to update
			GO_DOWN_STAGE_ONE_KEY_PREV=GO_DOWN_STAGE_ONE_KEY;			//don't forget to update
			
			
			
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
			if(ticks_msimg%200==0){
				for (uint8_t j=2;j<12;j++) tft_clear_line(j);
				//for(uint8_t i=0;i<4;i++)
					//tft_prints(1,i+2,"LMP %d",LiftingMotorPositionSetpoint[i]);
				//for(uint8_t i=0;i<4;i++)
					//tft_prints(1,i+6,"LMS %d",LiftingMotorSpeedSetpoint[i]);
				for(uint8_t i=0;i<4;i++)
					tft_prints(1,i+6,"ecd %d %f", i+1, LiftingMotorPositionFeedback[i]);
				for (int i=0;i<4;i++)
					tft_prints(1,i+2,"bfdSP%d %d",i+1, LiftingMotorSpeedSetpointBuffered[i]);
				
				//tft_prints(1,11,"On %b", GO_ON_STAGE_ONE_KEY);
				//tft_prints(1,12,"break %b", BREAK);
				//tft_prints(1,13,"BD %b FD %b", ONE_KEY_DOWN_BACK, ONE_KEY_DOWN_FRONT);
				tft_update();	
			}
			
			//
			//
			//
			//
			FRIC_SET_THRUST_L(700);
      FRIC_SET_THRUST_R(700);
			
			
			
			
			
		}//main loop with ticks	
	}
	
	
}	//main
