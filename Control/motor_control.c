
#include "motor_control.h"
#include "hw_elec.h"
#include "encoder_cali.h"
#include "MT6816Driver.h"


Motor_Control_Typedef 	motor_control;
extern MT6816_Typedef	mt6816;

int32_t Motor_Control_AdvanceCompen(int32_t _speed);

/****************************************  电流输出(电流控制)  ****************************************/
/**
  * @brief  电流输出
  * @param  current: 输出电流
  * @retval NULL
**/
void Control_Cur_To_Electric(int16_t current)
{
	//输出FOC电流
	motor_control.foc_current = current;
	//输出FOC位置
	if(motor_control.foc_current > 0)				motor_control.foc_location = motor_control.est_location + Move_Divide_NUM;
	else if(motor_control.foc_current < 0)	motor_control.foc_location = motor_control.est_location - Move_Divide_NUM;
	else																		motor_control.foc_location = motor_control.est_location;
	
	//输出任务到驱动
	motor_control.est_location = motor_control.foc_location;
	REIN_HW_Elec_SetDivideElec(motor_control.foc_location, motor_control.foc_current);
	//CurrentControl_Out_FeedTrack(motor_control.foc_location, motor_control.foc_current, false, true);
}



void Delay_us(uint16_t time)//Delay_us(0) = 10.2us
{
		HAL_SYSTICK_Config(72);
		HAL_Delay(time);
		HAL_SYSTICK_Config(72000);
}


/****************************************  PID控制(速度控制)  ****************************************/
/****************************************  PID控制(速度控制)  ****************************************/
//PID控制
Control_PID_Typedef pid;

/**
  * @brief  参数配置
  * @param  _k
  * @retval NULL
**/
void Control_PID_SetKP(uint16_t _k)
{
	if(_k <= 1024){		pid.kp = _k;		pid.valid_kp = true;		}
	else{															pid.valid_kp = false;		}
}

/**
  * @brief  参数配置
  * @param  _k
  * @retval NULL
**/
void Control_PID_SetKI(uint16_t _k)
{
	if(_k <= 1024){		pid.ki = _k;		pid.valid_ki = true;		}
	else{															pid.valid_ki = false;		}
}

/**
  * @brief  参数配置
  * @param  _k
  * @retval NULL
**/
void Control_PID_SetKD(uint16_t _k)
{
	if(_k <= 1024){		pid.kd = _k;		pid.valid_kd = true;		}
	else{															pid.valid_kd = false;		}
}

/**
  * @brief  PID参数恢复
  * @param  NULL
  * @retval NULL
**/
void Control_PID_Set_Default(void)
{
	Control_PID_SetKP(De_PID_KP);
	Control_PID_SetKI(De_PID_KI);
	Control_PID_SetKD(De_PID_KD);
}

/**
  * @brief  控制器PID初始化
  * @param  NULL
  * @retval NULL
**/
void Control_PID_Init(void)
{
	//前置配置无效时,加载默认配置
	if(!pid.valid_kp)				{	Control_PID_SetKP(De_PID_KP);		}
	if(!pid.valid_ki)				{	Control_PID_SetKI(De_PID_KI);		}
	if(!pid.valid_kd)				{	Control_PID_SetKD(De_PID_KD);		}
	
	//控制参数
	pid.v_error = 0;	pid.v_error_last = 0;
	pid.op = 0;				pid.oi = 0;			pid.od = 0;	
	pid.i_mut = 0;		pid.i_dec = 0;
	pid.out = 0;
}

/**
  * @brief  PID电流控制
  * @param  _speed    控制速度
  * @retval NULL
**/
void Control_PID_To_Electric(int32_t _speed)
{
	//误差
	pid.v_error_last = pid.v_error;
	pid.v_error = _speed - motor_control.est_speed;	//速度误差
	if(pid.v_error > ( 1024 * 1024))	pid.v_error = ( 1024 * 1024);
	if(pid.v_error < (-1024 * 1024))	pid.v_error = (-1024 * 1024);
	//op输出
	pid.op = ((pid.kp) * (pid.v_error));
	//oi输出
	pid.i_mut += ((pid.ki) * (pid.v_error));
	pid.i_dec  = (pid.i_mut >> 10);
	pid.i_mut -= (pid.i_dec << 10);
	pid.oi    += (pid.i_dec);
	if(pid.oi >      (  Current_Rated_Current << 10 ))	pid.oi = (  Current_Rated_Current << 10 );	//限制为额定电流 * 1024
	else if(pid.oi < (-(Current_Rated_Current << 10)))	pid.oi = (-(Current_Rated_Current << 10));	//限制为额定电流 * 1024
	//od输出
	pid.od = (pid.kd) * (pid.v_error - pid.v_error_last);
	//综合输出计算
	pid.out = (pid.op + pid.oi + pid.od) >> 10;
	if(pid.out > 			Current_Rated_Current)		pid.out =  Current_Rated_Current;
	else if(pid.out < -Current_Rated_Current)		pid.out = -Current_Rated_Current;
	
	//输出FOC电流
	motor_control.foc_current = pid.out;
	//输出FOC位置
	if(motor_control.foc_current > 0)				motor_control.foc_location = motor_control.est_location + Move_Divide_NUM;
	else if(motor_control.foc_current < 0)	motor_control.foc_location = motor_control.est_location - Move_Divide_NUM;
	else																		motor_control.foc_location = motor_control.est_location;
	//输出任务到驱动
	REIN_HW_Elec_SetDivideElec(motor_control.foc_location, motor_control.foc_current);
}


/****************************************  DCE控制器(位置控制)  ****************************************/
/****************************************  DCE控制器(位置控制)  ****************************************/
//DCE控制
Control_DCE_Typedef dce;

/**
  * @brief  参数配置
  * @param  _k
  * @retval NULL
**/
void Control_DCE_SetKP(uint16_t _k)
{
	if(_k <= 1024){		dce.kp = _k;		dce.valid_kp = true;		}
	else{															dce.valid_kp = false;		}
}

/**
  * @brief  参数配置
  * @param  _k
  * @retval NULL
**/
void Control_DCE_SetKI(uint16_t _k)
{
	if(_k <= 1024){		dce.ki = _k;		dce.valid_ki = true;		}
	else{															dce.valid_ki = false;		}
}

/**
  * @brief  参数配置
  * @param  _k
  * @retval NULL
**/
void Control_DCE_SetKV(uint16_t _k)
{
	if(_k <= 1024){		dce.kv = _k;		dce.valid_kv = true;		}
	else{															dce.valid_kv = false;		}
}

/**
  * @brief  参数配置
  * @param  _k
  * @retval NULL
**/
void Control_DCE_SetKD(uint16_t _k)
{
	if(_k <= 1024){		dce.kd = _k;		dce.valid_kd = true;		}
	else{															dce.valid_kd = false;		}
}

/**
  * @brief  DCE参数恢复
  * @param  NULL
  * @retval NULL
**/
void Control_DCE_Set_Default(void)
{
	Control_DCE_SetKP(De_DCE_KP);
	Control_DCE_SetKI(De_DCE_KI);
	Control_DCE_SetKV(De_DCE_KV);
	Control_DCE_SetKD(De_DCE_KD);
}

/**
  * @brief  控制器DCE初始化
  * @param  NULL
  * @retval NULL
**/
void Control_DCE_Init(void)
{
	//前置配置无效时,加载默认配置
	if(!dce.valid_kp)				{	Control_DCE_SetKP(De_DCE_KP);		}
	if(!dce.valid_ki)				{	Control_DCE_SetKI(De_DCE_KI);		}
	if(!dce.valid_kv)				{	Control_DCE_SetKV(De_DCE_KV);		}
	if(!dce.valid_kd)				{	Control_DCE_SetKD(De_DCE_KD);		}
	
	//控制参数(基本部分)
	dce.p_error = 0;	dce.v_error = 0;
	dce.op = 0;				dce.oi = 0;			dce.od = 0;	
	dce.i_mut = 0;		dce.i_dec = 0;
	dce.out = 0;
}

/**
  * @brief  DCE电流控制
  * @param  _location 控制位置
  * @param  _speed    控制速度
  * @retval NULL
**/
void Control_DCE_To_Electric(int32_t _location, int32_t _speed)
{
	//误差
	dce.p_error = _location - motor_control.est_location;
	dce.v_error = (_speed - motor_control.est_speed) >> 7;	//速度误差缩小至1/128
	if(dce.p_error > ( 3200))	dce.p_error = ( 3200);				//限制位置误差在1/16圈内(51200/16)
	if(dce.p_error < (-3200))	dce.p_error = (-3200);
	if(dce.v_error > ( 4000))	dce.v_error = ( 4000);				//限制速度误差在10r/s内(51200*10/128)
	if(dce.v_error < (-4000))	dce.v_error = (-4000);
	//op输出计算
	dce.op     = ((dce.kp) * (dce.p_error));
	//oi输出计算
	dce.i_mut += ((dce.ki) * (dce.p_error));
	dce.i_mut += ((dce.kv) * (dce.v_error));
	dce.i_dec  = (dce.i_mut >> 7);
	dce.i_mut -= (dce.i_dec << 7);
	dce.oi    += (dce.i_dec);
	if(dce.oi >      (  Current_Rated_Current << 10 ))	dce.oi = (  Current_Rated_Current << 10 );	//限制为额定电流 * 1024
	else if(dce.oi < (-(Current_Rated_Current << 10)))	dce.oi = (-(Current_Rated_Current << 10));	//限制为额定电流 * 1024
	//od输出计算
	dce.od = ((dce.kd) * (dce.v_error));
	//综合输出计算
	dce.out = (dce.op + dce.oi + dce.od) >> 10;
	if(dce.out > 			Current_Rated_Current)		dce.out =  Current_Rated_Current;
	else if(dce.out < -Current_Rated_Current)		dce.out = -Current_Rated_Current;

	//输出FOC电流
	motor_control.foc_current = dce.out;
	//输出FOC位置
	if(motor_control.foc_current > 0)			motor_control.foc_location = motor_control.est_location + 80;
	else if(motor_control.foc_current < 0)	motor_control.foc_location = motor_control.est_location - 80;
	else																		motor_control.foc_location = motor_control.est_location;
	//输出任务到驱动
	REIN_HW_Elec_SetDivideElec(motor_control.foc_location, motor_control.foc_current);
}

void Motor_Control_Callback(void)
{
	/************************************ 首次进入控制回调 ************************************/
	/************************************ 首次进入控制回调 ************************************/
	static bool first_call = true;
	if(first_call)
	{
		//读取(为了方便将XDrive代码移植到软件编码器,将位置读取初始化部分全部放置在此处运行)
		motor_control.real_lap_location				= mt6816.rectify_angle;
		motor_control.real_lap_location_last	= mt6816.rectify_angle;
		motor_control.real_location						= mt6816.rectify_angle;
		motor_control.real_location_last			= mt6816.rectify_angle;
		motor_control.goal_location           = mt6816.rectify_angle;
		//第一次运行强制退出
		first_call = false;
		return;
	}
	
	/************************************ 数据采集 ************************************/
	/************************************ 数据采集 ************************************/
	int32_t		sub_data;		//用于各个算差
	//读取单圈位置
	motor_control.real_lap_location_last = motor_control.real_lap_location;
	motor_control.real_lap_location = mt6816.rectify_angle;
	//回环检测
	sub_data = motor_control.real_lap_location - motor_control.real_lap_location_last;
	if(sub_data > (Move_Pulse_NUM >> 1))				sub_data -= Move_Pulse_NUM;
	else if(sub_data < -(Move_Pulse_NUM >> 1))	sub_data += Move_Pulse_NUM;
	//读取位置
	motor_control.real_location_last = motor_control.real_location;
	motor_control.real_location += sub_data;
	
	/************************************ 数据估计 ************************************/
	/************************************ 数据估计 ************************************/
	//估计速度
	motor_control.est_speed_mut += (	((motor_control.real_location - motor_control.real_location_last) * (CONTROL_FREQ_HZ))
																	+ ((int32_t)(motor_control.est_speed  << 5) - (int32_t)(motor_control.est_speed))
																	);
	motor_control.est_speed      = (motor_control.est_speed_mut >> 5);																	//(取整)(向0取整)(保留符号位)
	motor_control.est_speed_mut  = ((motor_control.est_speed_mut) - ((motor_control.est_speed << 5)));	//(取余)(向0取整)(保留符号位)
	//估计位置
	//motor_control.est_lead_location = Motor_Control_AdvanceCompen(motor_control.est_speed);
	motor_control.est_location = motor_control.real_location;// + motor_control.est_lead_location;
	//估计误差
	motor_control.est_error = motor_control.soft_location - motor_control.est_location;

	//Control_PID_To_Electric(100000);
	//Control_DCE_To_Electric(motor_control.goal_location, motor_control.goal_speed);
	
	
		/************************************ 运动控制 ************************************/
	/************************************ 运动控制 ************************************/
	if(0){}
	//输出休眠(禁用了输出IO,恢复输出需要单独处理)
	else if(
		 (motor_control.stall_flag)			//堵转标志置位
	|| (motor_control.soft_disable)		//软目标_失能指令
	){
		//Motor_Control_Clear_Integral();		//清除积分
		motor_control.foc_location = 0;		//清FOC位置
		motor_control.foc_current = 0;		//清FOC电流
		REIN_HW_Elec_SetSleep();					//驱动休眠
		//CurrentControl_OutSleep();			//XDrive采用硬件逻辑电流控制,自动休眠
	}
	//输出刹车
	else if(
		 (motor_control.soft_brake)			//软目标_刹车指令
	){
		//Motor_Control_Clear_Integral();		//清除积分
		motor_control.foc_location = 0;		//清FOC位置
		motor_control.foc_current = 0;		//清FOC电流
		REIN_HW_Elec_SetBrake();					//驱动刹车
		//CurrentControl_OutBrake();			//XDrive采用硬件逻辑电流控制,自动刹车
	}
	else{
		//运行模式分支
		switch(Motor_Mode_Digital_Location)//motor_control.mode_run
		{
			//停止
			case Control_Mode_Stop:						REIN_HW_Elec_SetBrake();				break;
			//CAN
			case Motor_Mode_Digital_Location:	Control_DCE_To_Electric(motor_control.goal_location, motor_control.goal_speed);				break;
			//其他非法模式
			default:	break;
		}
	}
	
	/************************************ 状态记录 ************************************/
	/************************************ 状态记录 ************************************/
	//统一的电机状态
	if(motor_control.mode_run == Control_Mode_Stop)	//停止模式
		motor_control.state = Control_State_Stop;
	else
	{
			if( (motor_control.real_location == motor_control.goal_location)
			 && (motor_control.est_speed == 0))
				motor_control.state = Control_State_Finish;		//目标匹配
			else
				motor_control.state = Control_State_Running;
	}
}


/**
  * @brief  超前角补偿
  * @param  _speed:补偿速度
  * @retval 补偿角度
**/
int32_t Motor_Control_AdvanceCompen(int32_t _speed)
{
	/******************** !!!!! 重要1：本补偿表提取自DPS系列代码                                                  !!!!! ********************/
	/******************** !!!!! 重要2：由于源于其他传感器数据，本补偿表并不完全适合TLE5012和MT6816                !!!!! ********************/

	int32_t compen;
	if(_speed < 0){
		if(_speed > -100000)				compen = 0;
		else if(_speed > -1300000)	compen = (((_speed +  100000) * 262) >> 20) -   0;
		else if(_speed > -2200000)	compen = (((_speed + 1300000) * 105) >> 20) - 300;
		else												compen = (((_speed + 2200000) *  52) >> 20) - 390;
		if(compen < -430)						compen = -430;
	}
	else{
		if(_speed < 100000)					compen = 0;																					//(      0,  0) ~ ( 100000,  0)
		else if(_speed <  1300000)	compen = (((_speed -  100000) * 262) >> 20) +   0;	//( 100000,  0) ~ (1300000,300)
		else if(_speed <  2200000)	compen = (((_speed - 1300000) * 105) >> 20) + 300;	//(1300000,300) ~ (2200000,390)
		else												compen = (((_speed - 2200000) *  52) >> 20) + 390;	//(2200000,390) ~ 
		if(compen > 430)						compen = 430;
	}
	return compen;
}
