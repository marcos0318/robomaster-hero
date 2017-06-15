本实验为新建工程实验，仅供大家新建工程时参考。
新建工程详细步骤，请看《STM32F4开发指南-库函数版本》第3.3节。


               	正点原子@ALIENTEK
               	2014-10-24
		广州市星翼电子科技有限公司
                电话：020-38271790
                传真：020-36773971
	       	购买：http://shop62057469.taobao.com
                公司网站：www.alientek.com
         	技术论坛：www.openedv.com



int16_t checkSetpoint(int16_t a, bool dir){

	if(dir) 
		if(a > LiftingMotorSetpointLimit - 200) a = LiftingMotorSetpointLimit - 1;
		else 																		a += 200;
	
	else 
		if(a < 200) 														a = 0;
		else 																		a -= 200;
	
	return a;
}





else if(DBUS_CheckPush(KEY_Z)){
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = checkSetpoint(LiftingMotorSetpoint[0], true);
				DataMonitor_Send(0, LiftingMotorSetpoint[0]);
			}
			else if(DBUS_CheckPush(KEY_X)){
				LiftingMotorSetpoint[0] = LiftingMotorSetpoint[1] = checkSetpoint(LiftingMotorSetpoint[0], false);
				DataMonitor_Send(0, LiftingMotorSetpoint[0]);
			}
			else if(DBUS_CheckPush(KEY_C)){
				LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = checkSetpoint(LiftingMotorSetpoint[2], true);
				DataMonitor_Send(2, LiftingMotorSetpoint[2]);
			}
			else if(DBUS_CheckPush(KEY_V)){
				LiftingMotorSetpoint[2] = LiftingMotorSetpoint[3] = checkSetpoint(LiftingMotorSetpoint[2], false);
				DataMonitor_Send(2, LiftingMotorSetpoint[2]);
			}
			else if(!lower_pneumatic_prev && DBUS_CheckPush(KEY_Q)){
				//pneumatic
				lower_pneumatic_state = !lower_pneumatic_state;
				pneumatic_control(1, lower_pneumatic_state);
				pneumatic_control(2, lower_pneumatic_state);
			}
			else if(!upper_pneumatic_prev && DBUS_CheckPush(KEY_E)){
				//pneumatic
				if(upper_pneumatic_state == 0){
					DataMonitor_Send(26, 0);	// turn off friction wheel
					upper_pneumatic_state = 1;
					pneumatic_control(3, true);	//extend the pneumatic
				}
				else if(upper_pneumatic_state == 1){
					DataMonitor_Send(27, 0);	//friction wheel still off
					upper_pneumatic_state = 2;	//pneumatic still extended
				}
				else if(upper_pneumatic_state == 2){
					DataMonitor_Send(28,0);		//turn on friction wheel
					upper_pneumatic_state = 0;
					pneumatic_control(3, false);	//withdraw pneumatic
				}
				else
					DataMonitor_Send(0x55, 0);	//keep communication
			}