#include "hero_param.h"


int32_t DBUSBrokenLineRecover = 0;
int32_t CAN1BrokenLineRecover = 0;
int32_t CAN2BrokenLineRecover = 0;


int32_t GimbalFlag = 3;  
int32_t ChasisFlag = 1;
int32_t CameraFlag = 1;
int32_t ChasisFlag_Prev = 1;

int32_t LastDBUSLeftSwitch = 0;
int32_t LastDBUSRightSwitch = 0;

float kp_chassisAngle = 2.7;
float ki_chassisAngle = 0;
float kd_chassisAngle = 1;

//int32_t powerPID[3] = {10, 3, 0};
int32_t kp_power = 10;
int32_t ki_power = 3;
int32_t kd_power = 0;

//DBUS for gimbal
int32_t direction = 0;
int32_t direction_buffered = 0;
int32_t upperTotal = 360 * 27;

int32_t xtotal = 0;
int32_t pre_xtotal = 0;

//Dbus for chasis
int32_t xtotal_chasis = 0;
int32_t xtotal_chasis_prev = 0;

//float gimbalPositionPID[3] = {0.5, 0.00032, 22};
float kp_gimbalPosition = 1.2;
float ki_gimbalPosition = 0.00001;
float kd_gimbalPosition = 1;

//float pitchPositionPID[3] = {0.4, 0.0003, 12};
float kp_pitchPosition = 0.4;
float ki_pitchPosition = 0.000005;
float kd_pitchPosition = 1;

//float cameraPositionPID[3] = {0.3, 0.00, 1};
float kp_cameraPosition = 0.3;
float ki_cameraPosition = 0.00;
float kd_cameraPosition = 0.1;

//int32_t cameraSpeedPID[3] = {80, 4, 1};
int32_t kp_cameraSpeed = 100;
int32_t ki_cameraSpeed = 4;
int32_t kd_cameraSpeed = 1;

//The control of filter rate of wheels 
// Structure to strore PID data 
struct pid_control_states states[4] = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};
int32_t wheel_setpoints[4] = {0,0,0,0};
float wheel_setpoints_buffered[4] = {0,0,0,0};
int32_t wheel_setpoints_round[4] = {0,0,0,0};
int32_t wheel_feedbacks[4] = {0,0,0,0};
int32_t wheel_outputs[4] = {0,0,0,0};

int32_t kp = 80, ki = 4, kd = 1; // What is the physical meaning of this one?

//The control of angle of chasis
struct fpid_control_states state_angle = {0,0,0};
float setpoint_angle 	= 0;
float feedback_angle = 0;
int32_t output_angle_speed = 0;
int32_t output_angle_speed_buffered = 0;



/*****************************/
/***** DBUS control data *****/
/*****************************/

int32_t speed_limitor = 660;
int32_t angular_speed_limitor = FOR_JOHN_MAX_TURNING_SPEED;
int32_t forward_speed;
int32_t right_speed;
float corrected_forward_speed;
float corrected_right_speed;
int32_t increment_of_angle;
int32_t mouse_prev = 0;
float gimbalNotOutGyroOutput = 0;
bool locked = false;   //for key F
bool Fprev = false;


/*************************/
/***** Power Control *****/
/*************************/

// NOT intend to use pid control, but still consider using close-loop control

int32_t filter_rate_limit = FOR_JOHN_MAX_RUNNING_SPEED;
int32_t speed_multiplier = FOR_JOHN_MAX_RUNNING_SPEED;
int32_t RC_dir_multiplier = 1;

//int32_t power_buffer[POWER_BUFFER_LENGTH];
float feedback_current = 0;
float feedback_voltage = 0;
int32_t work_target = 60;
float Pr = 0;
float PL = 80;
float W = 60;
int32_t W_int = 60;
int32_t work_pid_output = 0;
int32_t wheel_setpoint_coefficient = 1000;
struct pid_control_states work_state = {0,0,0};


/******************************/
/*********** Camera ***********/
/******************************/

int32_t cameraPositionId = 0;
int32_t pressCameraChangePrev = 0;

float cameraPositionFeedback = 0;
float cameraPositionSetpoint = 0;
float cameraPositionOutput = 0;

struct fpid_control_states cameraPositionState = {0,0,0};

int32_t cameraSpeedSetpoint = 0;
int32_t cameraSpeedFeedback = 0;
int32_t cameraSpeedOutput = 0;

struct pid_control_states cameraSpeedState = {0,0,0};
float cameraArray[6] = {0, 2430, 2 * 2430, 3 * 2430, 2 * 2430, 2430};


/****************/
/***** Rune *****/
/****************/

int32_t isRuneMode = 0;
int32_t lastIsRuneMode = 0;

int32_t currentLeft = 0;
int32_t lastLeft = 0;


/*****************/
/***** Debug *****/
/*****************/

int32_t error = 0 ;
int32_t spSp = 0; 
int32_t Cerror = 0;


/* Upper Pneumatic State */


/* To be optimized... */
int16_t LiftingMotorSetpoint[4] = {0};
int32_t buffer[4][BUFFER_LENGTH];


/**********************************/
/***** Broken line protection *****/
/**********************************/
volatile uint8_t CAN1BrokenLine = 1;
volatile uint8_t CAN2BrokenLine = 1;
volatile uint8_t DBUSBrokenLine = 0;

volatile uint8_t CAN1BrokenLine_prev = 1;
volatile uint8_t CAN2BrokenLine_prev = 1;
volatile uint8_t DBUSBrokenLine_prev = 1;

volatile uint32_t CAN1BrokenLineCounter = 0;
volatile uint32_t CAN2BrokenLineCounter = 0;
volatile uint32_t DBUSBrokenLineCounter = 0;

volatile uint32_t Wheel1BrokenLineCounter = 1;
volatile uint32_t Wheel2BrokenLineCounter = 1;
volatile uint32_t Wheel3BrokenLineCounter = 1;
volatile uint32_t Wheel4BrokenLineCounter = 1;

volatile uint32_t YawBrokenLineCounter = 1;
volatile uint32_t PitchBrokenLineCounter = 1;
volatile uint32_t GunBrokenLineCounter = 1;
volatile uint32_t CameraBrokenLineCounter = 1;
