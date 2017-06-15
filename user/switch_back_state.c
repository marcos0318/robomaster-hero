#include "switch_back_state.h"

//Back_To_RUNNING_MODE: upper horizontal pneumatic and LiftingMotor timer and flag
volatile u8 B_RUNNING_MODE_UHPneu_LM_flag = 0;
volatile u32 B_RUNNING_MODE_UHPneu_LM_timer = 0;

//Back_To_DANCING_MODE: upper vertical pneumatic and friction wheel timer and flag
volatile u8 B_DANCING_MODE_UVPneu_FW_flag = 0;
volatile u32 B_DANCING_MODE_UVPneu_FW_timer = 0;

void Back_To_RUNNING_MODE()
{
    //set flags and speed limit and direction
    ChasisFlag = 1;     //turn on gyro
    GimbalFlag = 3;     //recover gimbal control
    filter_rate_limit = FOR_JOHN_MAX_RUNNING_SPEED;
    speed_multiplier = FOR_JOHN_MAX_RUNNING_SPEED;
    //QWEASD back to normal control direction

    //first withdraw the lower pneumatic and upper vertical pneumatic
    lower_pneumatic_state = 0;
    pneumatic_control(1, 0);
    pneumatic_control(2, 0);

    //need to delay DataMonitor_Send and upper horizontal pneumatic
    B_RUNNING_MODE_UHPneu_LM_flag = 1;
    B_RUNNING_MODE_UHPneu_LM_timer = TIM_7_Counter;

}

void Back_To_INTO_RI_MODE()
{
    LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = UP_SETPOINT/8; 
    DataMonitor_Send(0xFF, 0);

    //reverse QWEASD
    filter_rate_limit = FOR_JOHN_INTO_RI_MAX_SPEED;
    speed_multiplier = -FOR_JOHN_INTO_RI_MAX_SPEED;
    //turn off gyro
    ChasisFlag = 3;
}

void Back_To_BACK_WHEEL_UP()
{
    //gyro still off
    ChasisFlag = 3;
    LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = DOWN_SETPOINT/8;
    LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = UP_SETPOINT/8;
    DataMonitor_Send(70, 0);
}

void Back_To_FRONT_WHEEL_UP()
{
    //extend lower pneumatic
    pneumatic_control(1, 1);
    pneumatic_control(2, 1);
    //reverse QWEASD
    filter_rate_limit = FOR_JOHN_INTO_RI_MAX_SPEED;
    speed_multiplier = -FOR_JOHN_INTO_RI_MAX_SPEED;
    //turn off gyro
    ChasisFlag = 3;

    LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = DOWN_SETPOINT/8;
   DataMonitor_Send(71, 0); 

}

void Back_To_SPEED_LIMITATION()
{
    pneumatic_control(3, false);
    upper_pneumatic_state = 0;
    lower_pneumatic_state = false;
    pneumatic_control(1, 0);
    pneumatic_control(2, 0);
}

void Back_To_UPPER_HORIZONTAL_PNEUMATIC_EXTENDS()
{
    //turn off friction wheel
    DataMonitor_Send(26, 0);
    //withdraw vertical pneumatic
    pneumatic_control(4, 0);
}

void Back_To_CATCH_GOLF()
{
    //LiftingMotors stop oscillate
    //but friction wheel still on
    DataMonitor_Send(64, 1);
		//LiftinMotors need to go back to the highest point

}

void Back_To_DANCING_MODE()
{
		//need to first extend upper horizontal pneumatic
		pneumatic_control(3, true);
    ChasisFlag = 3;
    filter_rate_limit = FOR_JOHN_INTO_RI_MAX_SPEED;
    speed_multiplier = -FOR_JOHN_INTO_RI_MAX_SPEED;
    lower_pneumatic_state = false;
    pneumatic_control(1, 0);
    pneumatic_control(2, 0);
		DataMonitor_Send(64, 0);
}

void Back_To_VERTICAL_PNEUMATIC_WITHDRAWS()
{
    LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
    DataMonitor_Send(5, 0);
}

void Back_To_DOWN_FRONT_WHEEL()
{
    LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = UP_SETPOINT/8;
    LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = 0;
    DataMonitor_Send(70, 0);
}


void dummy()
{}

void (*backState[]) (void) = {
    dummy,
    Back_To_RUNNING_MODE,
    Back_To_INTO_RI_MODE,
    Back_To_BACK_WHEEL_UP,
    Back_To_FRONT_WHEEL_UP,
    Back_To_SPEED_LIMITATION,
    Back_To_UPPER_HORIZONTAL_PNEUMATIC_EXTENDS,
    Back_To_CATCH_GOLF,
    Back_To_DANCING_MODE,
    Back_To_VERTICAL_PNEUMATIC_WITHDRAWS,
    Back_To_DOWN_FRONT_WHEEL,
};
