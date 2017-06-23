#include "chasis_control.h"
#include "gimbal_control.h"
#include "slave_communication.h"
#include <math.h>

void DBUS_data_analysis(){
	speed_limitor  = 660;
	//speed_multiplier = filter_rate_limit;
	angular_speed_limitor = FOR_JOHN_MAX_TURNING_SPEED;
	if(HERO == RUNNING_MODE){
		if(DBUS_CheckPush(KEY_W)||DBUS_CheckPush(KEY_A)||DBUS_CheckPush(KEY_S)||DBUS_CheckPush(KEY_D)){
			if(DBUS_CheckPush(KEY_CTRL)){	//control
				filter_rate_limit = FOR_JOHN_CTRL_MAX_RUNNING_SPEED;
				speed_multiplier= FOR_JOHN_CTRL_MAX_RUNNING_SPEED;
			}
			else if(!DBUS_CheckPush(KEY_SHIFT)){	//normal
				filter_rate_limit = FOR_JOHN_MAX_RUNNING_SPEED;
				speed_multiplier= FOR_JOHN_MAX_RUNNING_SPEED;
			}
			else if(DBUS_CheckPush(KEY_SHIFT)){	//shift
				filter_rate_limit = FOR_JOHN_SHIFT_MAX_RUNNING_SPEED;
				speed_multiplier= FOR_JOHN_SHIFT_MAX_RUNNING_SPEED;
			}
		}
			
	}
	forward_speed = (DBUS_ReceiveData.rc.ch1 + DBUS_CheckPush(KEY_W)*660 - DBUS_CheckPush(KEY_S)*660) * speed_multiplier/speed_limitor;
	right_speed =   (DBUS_ReceiveData.rc.ch0 + DBUS_CheckPush(KEY_D)*660 - DBUS_CheckPush(KEY_A)*660) * speed_multiplier/speed_limitor;
	
	//Correction of bias due to the gimbal and the chasis are not in the same dirction.
	//The direction of moving should be as the same as the gimbal, as the camera is on the gimbal
	double bias_angle = - GMYawEncoder.ecd_angle * 2 * 3.1415936 / upperTotal;
	
	
	
	corrected_forward_speed = (float)forward_speed * cos(bias_angle) - (float)right_speed * sin(bias_angle); 
	corrected_right_speed = (float)right_speed * cos(bias_angle) + (float)forward_speed * sin(bias_angle);
	
	
	
	
	//assign value back in order not to change the API
	forward_speed =  corrected_forward_speed;
	right_speed = corrected_right_speed;
	
	
	
	
}


void turning_speed_limit_control(uint32_t ticks_msimg){
	xtotal_chasis = DBUS_ReceiveData.mouse.xtotal;

	feedback_angle = output_angle;
				
	output_angle_speed = pid_process(&state_angle,&setpoint_angle, &feedback_angle, kp_chassisAngle, ki_chassisAngle, kd_chassisAngle);
	
	windowLimit(&output_angle_speed, FOR_JOHN_MAX_TURNING_SPEED, -FOR_JOHN_MAX_TURNING_SPEED);

	int32_t max_wheel_setpoint = abs(forward_speed) + abs(right_speed);	

	int32_t larger_abs_speed = max(abs(forward_speed), abs(right_speed));
	
	wheel_setpoints[0] = (- forward_speed - right_speed) * larger_abs_speed / max_wheel_setpoint ;
	wheel_setpoints[1] = (  forward_speed - right_speed) * larger_abs_speed / max_wheel_setpoint ;
	wheel_setpoints[2] = (  forward_speed + right_speed) * larger_abs_speed / max_wheel_setpoint ;
	wheel_setpoints[3] = (- forward_speed + right_speed) * larger_abs_speed / max_wheel_setpoint ;
	//but the code above is only for the moving back and forth, left and right
					
	for (int i=0; i<4; i++){
		buffer_in(buffer[i], BUFFER_LENGTH, ticks_msimg , wheel_setpoints[i]);
		wheel_setpoints[i] = buffer_out(buffer[i], BUFFER_LENGTH, ticks_msimg);

		if (ChasisFlag == 1 || ChasisFlag == 2)
			wheel_setpoints[i] -= output_angle_speed;
		if (ChasisFlag  == 3)
			wheel_setpoints[i] -= DBUS_ReceiveData.mouse.x * 15;

		//Use Q and E to control direction
		if (ChasisFlag == 4 || ChasisFlag == 3) {
			if(DBUS_CheckPush(KEY_Q))
				wheel_setpoints[i] += FOR_JOHN_QE_INC;
			if(DBUS_CheckPush(KEY_E))
				wheel_setpoints[i] -= FOR_JOHN_QE_INC;
			if(LeftJoystick)
				wheel_setpoints[i] -= DBUS_ReceiveData.rc.ch2 / 5;
		}

		xtotal_chasis_prev = xtotal_chasis;

	}	
	
		
		
	wheel_setpoints_adjust(&wheel_setpoints[0], &wheel_setpoints[1],&wheel_setpoints[2],&wheel_setpoints[3] , filter_rate_limit);
	
	for (int i=0; i<4; i++) {
		if ( wheel_setpoints[i] > wheel_setpoints_buffered [i]) {
			wheel_setpoints_buffered[i] += 1;
		}
		else if ( wheel_setpoints[i] < wheel_setpoints_buffered [i] ) {
			wheel_setpoints_buffered[i] -= 1;
		}
	}
	
	//these are the feed back as the current state 
	wheel_feedbacks[0] = CM1Encoder.filter_rate;
	wheel_feedbacks[1] = CM2Encoder.filter_rate;
	wheel_feedbacks[2] = CM3Encoder.filter_rate;
	wheel_feedbacks[3] = CM4Encoder.filter_rate;

	//pid process to get the output as the torque
	if ( GimbalFlag != 1 ) {
	  for (int i=0; i<4; i++) {
			wheel_outputs[i] = pid_process(&states[i], &wheel_setpoints_buffered[i], &wheel_feedbacks[i], kp, ki, kd);
		}
	    
	}
	else {
	  for (int i=0; i<4; i++)
	    wheel_outputs[i] = 0;
	}



}

