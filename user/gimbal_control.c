#include "gimbal_control.h"
void gimbal_yaw_control(){
	isGimbalPositionSetpointIncrease = (bufferedGimbalPositionSetpoint < gimbalPositionSetpoint);

	if(isGimbalPositionSetpointIncrease){
		bufferedGimbalPositionSetpoint += 5;

		if (bufferedGimbalPositionSetpoint > gimbalPositionSetpoint)
			bufferedGimbalPositionSetpoint = gimbalPositionSetpoint;
	}
	else {
		bufferedGimbalPositionSetpoint -= 5;

		if(bufferedGimbalPositionSetpoint < gimbalPositionSetpoint) 
			bufferedGimbalPositionSetpoint = gimbalPositionSetpoint;
	}
	
	
	
	gimbalPositionFeedback = GMYawEncoder.ecd_angle;
	gimbalSpeedSetpoint = (int32_t)fpid_process(&gimbalPositionState, &bufferedGimbalPositionSetpoint, &gimbalPositionFeedback, kp_gimbalPosition, ki_gimbalPosition, kd_gimbalPosition);
	//Limit the output
	windowLimit(&gimbalSpeedSetpoint, 80, -80);
	
	//Get the speed here
	incPIDsetpoint(&gimbalSpeedMoveState, gimbalSpeedSetpoint);
	gimbalSpeedMoveOutput += incPIDcalc(&gimbalSpeedMoveState, GMYawEncoder.filter_rate);

}

void gimbal_pitch_control(){
	//limit pitch position
	windowLimit(&DBUS_ReceiveData.mouse.ytotal, -460/pitchPosMultiplier, -1100/pitchPosMultiplier);
	//pitch setpoint
	pitchPositionSetpoint = -DBUS_ReceiveData.mouse.ytotal * pitchPosMultiplier;

	isPitchPositionSetpointIncrease = (bufferedPitchPositionSetpoint < pitchPositionSetpoint);
	
	if(isPitchPositionSetpointIncrease) {
		bufferedPitchPositionSetpoint += 70;

		if (bufferedPitchPositionSetpoint > pitchPositionSetpoint)
			bufferedPitchPositionSetpoint = pitchPositionSetpoint;
	}
	else {
		bufferedPitchPositionSetpoint -= 70;

		if(bufferedPitchPositionSetpoint < pitchPositionSetpoint) 
			bufferedPitchPositionSetpoint = pitchPositionSetpoint;
	}
	
	pitchPositionFeedback = GMPitchEncoder.ecd_angle;
	pitchSpeedSetpoint = (int32_t)fpid_process(&pitchPositionState, &pitchPositionSetpoint, &pitchPositionFeedback, kp_pitchPosition, ki_pitchPosition, kd_pitchPosition);
	
	windowLimit(&pitchSpeedSetpoint, 80, -80);
	
	incPIDsetpoint(&pitchSpeedMoveState, pitchSpeedSetpoint);
	pitchSpeedMoveOutput += incPIDcalc(&pitchSpeedMoveState, GMPitchEncoder.filter_rate);
}