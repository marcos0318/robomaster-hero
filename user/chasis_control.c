#include "chasis_control.h"
void camera_position_control(){
	if (cameraPositionId == 0 && DBUS_CheckPush(KEY_CTRL) == 0) 
		filter_rate_limit = 600;
	else 
		filter_rate_limit = 200;

	if (pressCameraChangePrev == 0 && DBUS_CheckPush(KEY_Q)){
		cameraPositionId++;
		if (cameraPositionId == 6)
			cameraPositionId = 0;
		cameraPositionSetpoint = cameraArray[cameraPositionId];
	}
	cameraPositionFeedback = GMCameraEncoder.ecd_angle;
	cameraPositionOutput = fpid_process(&cameraPositionState, &cameraPositionSetpoint, &cameraPositionFeedback, kp_cameraPosition, ki_cameraPosition, kd_cameraPosition);

	cameraSpeedSetpoint = (int32_t) cameraPositionOutput;

	cameraSpeedFeedback = GMCameraEncoder.filter_rate;
	cameraSpeedOutput = pid_process(&cameraSpeedState, &cameraSpeedSetpoint, &cameraSpeedFeedback, kp_cameraSpeed, ki_cameraSpeed, kd_cameraSpeed);

}																													

void Dbus_data_analysis(){
	speed_limitor  = 660;
	speed_multiplier = filter_rate_limit;
	angular_speed_limitor = 200;
	forward_speed = (DBUS_ReceiveData.rc.ch1 + DBUS_CheckPush(KEY_W)*660 - DBUS_CheckPush(KEY_S)*660) * speed_multiplier/speed_limitor;
	right_speed =   (DBUS_ReceiveData.rc.ch0 + DBUS_CheckPush(KEY_D)*660 - DBUS_CheckPush(KEY_A)*660) * speed_multiplier/speed_limitor;

}

void keyboard_mouse_control(){
	xtotal =  DBUS_ReceiveData.mouse.xtotal;
	//direction not move when the difference is large
	if (abs(direction + output_angle*upperTotal / 3600) <= outsideLimit) 
		direction += (-DBUS_ReceiveData.rc.ch2 / 300 + -(xtotal - pre_xtotal)*7);
	else if ((direction + output_angle*upperTotal / 3600) > outsideLimit)
		direction = outsideLimit - output_angle * upperTotal/3600;			
	else if ((direction + output_angle * upperTotal / 3600) < - outsideLimit)
		direction = -outsideLimit - output_angle * upperTotal / 3600;

	gimbalPositionSetpoint = direction +  output_angle*upperTotal/3600;

	if(DBUS_ReceiveData.mouse.press_right || abs(DBUS_ReceiveData.rc.ch2)>3){
		setpoint_angle = -direction * 3600/upperTotal;
	}

	//Used for protection				
	
	//windowLimit(&gimbalPositionSetpoint, 700, -700); //problem with overloading, write another function
	if(gimbalPositionSetpoint > 700)
		gimbalPositionSetpoint = 700;
	else if (gimbalPositionSetpoint < -700)
		gimbalPositionSetpoint = -700;
	
	// else gimbalPositionSetpoint=-DBUS_ReceiveData.mouse.xtotal*yawPosMultiplier;

	pre_xtotal = xtotal;
}

void turning_speed_limit_control(){
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

		if(cameraPositionId == 0)
			wheel_setpoints[i] += output_angle_speed;
		else 
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

