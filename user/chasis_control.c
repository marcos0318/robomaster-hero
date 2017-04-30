#include "chasis_control.h"
#include "gimbal_control.h"



void DBUS_data_analysis(){
	speed_limitor  = 660;
	//speed_multiplier = filter_rate_limit;
	angular_speed_limitor = 200;
	forward_speed = (DBUS_ReceiveData.rc.ch1 + DBUS_CheckPush(KEY_W)*660 - DBUS_CheckPush(KEY_S)*660) * speed_multiplier/speed_limitor;
	right_speed =   (DBUS_ReceiveData.rc.ch0 + DBUS_CheckPush(KEY_D)*660 - DBUS_CheckPush(KEY_A)*660) * speed_multiplier/speed_limitor;
	/*
	if (DBUS_ReceiveData.rc.switch_left == 1) {
		if (DBUS_ReceiveData.mouse.press_right) 
			ChasisFlag = 2;
		else 
			ChasisFlag = 1;
	}
	else if (DBUS_ReceiveData.rc.switch_left == 3) {
		if (DBUS_ReceiveData.mouse.press_right) 
			ChasisFlag = 4;
		else
			ChasisFlag = 3;
	}
	*/
}


void turning_speed_limit_control(uint32_t ticks_msimg){
	feedback_angle = output_angle;
				
	output_angle_speed = pid_process(&state_angle,&setpoint_angle, &feedback_angle, kp_chassisAngle, ki_chassisAngle, kd_chassisAngle);
	windowLimit(&output_angle_speed, CHASSIS_ANGULAR_VELOCITY_LIMIT, -CHASSIS_ANGULAR_VELOCITY_LIMIT);

	int32_t max_wheel_setpoint = abs(forward_speed) + abs(right_speed);	

	int32_t larger_abs_speed = max(abs(forward_speed), abs(right_speed));
	
	wheel_setpoints[0] = (  forward_speed + right_speed) * larger_abs_speed / max_wheel_setpoint ;
	wheel_setpoints[1] = (- forward_speed + right_speed) * larger_abs_speed / max_wheel_setpoint ;
	wheel_setpoints[2] = (- forward_speed - right_speed) * larger_abs_speed / max_wheel_setpoint ;
	wheel_setpoints[3] = (  forward_speed - right_speed) * larger_abs_speed / max_wheel_setpoint ;
	//but the code above is only for the moving back and forth, left and right
					
	for (int i=0; i<4; i++){
		buffer_in(buffer[i], BUFFER_LENGTH, ticks_msimg , wheel_setpoints[i]);
		wheel_setpoints[i] = buffer_out(buffer[i], BUFFER_LENGTH, ticks_msimg);

		if(ChasisFlag == 1 || ChasisFlag == 2)
			wheel_setpoints[i] += output_angle_speed;
		if (ChasisFlag  == 3)
			wheel_setpoints[i] += DBUS_ReceiveData.mouse.x * 6;
	}	
	
	for(int i=0; i<4; i++)
			wheel_setpoints[i] = wheel_setpoints[i] * wheel_setpoint_coefficient / 1000;
		
		
	wheel_setpoints_adjust(&wheel_setpoints[0], &wheel_setpoints[1],&wheel_setpoints[2],&wheel_setpoints[3] , filter_rate_limit);
	
	//these are the feed back as the current state 
	wheel_feedbacks[0] = CM1Encoder.filter_rate;
	wheel_feedbacks[1] = CM2Encoder.filter_rate;
	wheel_feedbacks[2] = CM3Encoder.filter_rate;
	wheel_feedbacks[3] = CM4Encoder.filter_rate;

	//pid process to get the output as the torque
	for (int i=0; i<4; i++)
		wheel_outputs[i] = pid_process(&states[i], &wheel_setpoints[i], &wheel_feedbacks[i], kp, ki, kd);
}

