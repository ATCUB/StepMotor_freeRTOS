
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "struct_typedef.h"
#include "main.h"


#define Move_Divide_NUM			((int32_t)(256))		

#define Current_Rated_Current		(3000)		//额定电流(mA)
#define Current_Cali_Current		(1500)		//校准电流(mA)

/****************************************  控制器频率配置区  ****************************************/
#define CONTROL_FREQ_HZ			(500)												//控制频率_hz
#define CONTROL_PERIOD_US		(1000000 / CONTROL_FREQ_HZ)		//控制周期_us

typedef struct{
	//配置
	#define De_PID_KP	5		//默认KP
	#define De_PID_KI	30	//默认KI
	#define De_PID_KD	0		//默认KD
	bool		valid_kp, valid_ki, valid_kd;	//参数有效标志
	int32_t	kp, ki, kd;		//参数
	//控制参数
	int32_t		v_error, v_error_last;	//误差记录
	int32_t		op, oi, od;							//输出
	int32_t		i_mut, i_dec;						//小积分处理
	int32_t		out;										//输出
}Control_PID_Typedef;

typedef struct{
	//配置
	#define De_DCE_KP	400		//默认KP
	#define De_DCE_KI	200		//默认KI
	#define De_DCE_KV	650		//默认KIV
	#define De_DCE_KD	200		//默认KD
	bool		valid_kp, valid_ki, valid_kv, valid_kd;	//参数有效标志
	int32_t	kp, ki, kv, kd;		//参数
	//控制参数(基本部分)
	int32_t		p_error, v_error;		//误差记录
	int32_t		op, oi, od;					//输出
	int32_t		i_mut, i_dec;				//小积分处理
	int32_t		out;								//输出
}Control_DCE_Typedef;

/****************************************  Motor_Control  ****************************************/
/**
  * 控制器状态
**/
typedef enum{
	Control_State_Stop				= 0x00,	//停止
	Control_State_Finish			= 0x01,	//任务完成
	Control_State_Running			= 0x02,	//任务执行中
	Control_State_Overload		= 0x03,	//过载
	Control_State_Stall				= 0x04,	//堵转
}Motor_State;


/**
  * 模式
**/
typedef enum{
	/*****************************************************外部输入量***********信号发生器(可控的运动参数)**开放状态(开放版本)*****/
	//测试
	Motor_Mode_Debug_Location		= 0x00,	//调试位置				NULL									多功能调试信号源						测试版本完整代码
	Motor_Mode_Debug_Speed			= 0x01,	//调试速度				NULL									多功能调试信号源						测试版本完整代码
	//停止
	Control_Mode_Stop						= 0x10,	//停止						NULL									NULL												NULL
	//DIG(CAN/RS485)
	Motor_Mode_Digital_Location	= 0x20,	//DIG位置					目标位置							位置跟踪器(速度,双加速度)		源码开放(20191101)
	Motor_Mode_Digital_Speed		= 0x21,	//DIG速度					目标速度							速度跟踪器(双加速度)				源码开放(20191101)
	Motor_Mode_Digital_Current	= 0x22,	//DIG电流					目标电流							电流跟踪器(双电梯度)				源码开放(20191101)
	Motor_Mode_Digital_Track		= 0x23,	//DIG轨迹					多指令								运动重构器(运动自寻)				源码开放(20191101)
	//MoreIO(PWM/PUL)
	Motor_Mode_PWM_Location			= 0x30,	//PWM位置_舵机		目标位置							位置跟踪器(速度,双加速度)		源码开放(20191101)
	Motor_Mode_PWM_Speed				= 0x31,	//PWM速度_调速机	目标速度							速度跟踪器(双加速度)				源码开放(20191101)
	Motor_Mode_PWM_Current			= 0x32,	//PWM电流_直流机	目标电流							电流跟踪器(双电梯度)				源码开放(20191101)
	Motor_Mode_PULSE_Location		= 0x33,	//PULSE位置				目标位置							位置插补器(无)							源码开放(20200803)
}Motor_Mode;


/**
  * Motor_Control类结构体定义
**/
typedef struct{
	//配置(模式)
	#define			De_Motor_Mode		Motor_Mode_PULSE_Location	//默认配置
	bool				valid_mode;		//有效标志
	Motor_Mode	mode_order;		//电机模式_新指令的
	//配置(堵转)
	#define			De_Motor_Stall	true		//默认堵转保护开关
	bool				valid_stall_switch;			//堵转保护开关有效标志
	bool				stall_switch;						//堵转保护开关
	//模式
	Motor_Mode	mode_run;			//电机模式_运行中的
	//读取
	int32_t		real_lap_location;				//读取单圈位置
	int32_t		real_lap_location_last;		//读取单圈位置
	int32_t		real_location;						//读取位置
	int32_t		real_location_last;				//读取位置
	//估计
	int32_t		est_speed_mut;						//估计速度倍值(放大n倍)(mut有积分作用,不会有精度损失)
	int32_t		est_speed;								//估计速度
	int32_t		est_lead_location;				//估计位置超前位置
	int32_t		est_lead_location_debug;	//估计位置超前位置
	int32_t		est_location;							//估计位置
	int32_t		est_error;								//估计位置误差
	//硬目标
	int32_t		goal_location;	  //目标位置(由信号输入)
	int32_t		goal_speed;	  	//目标速度(由信号输入)
	int16_t		goal_current;		 //目标电流(由信号输入)
	bool			goal_disable;		   //目标失能(由信号输入)
	bool			goal_brake;		     //目标刹车(由信号输入)
	//软目标
	int32_t		soft_location;	//软位置(由 跟踪器/重构器/插补器/硬运算 得到)
	int32_t		soft_speed;			//软速度(由 跟踪器/重构器/插补器/硬运算 得到)
	int16_t		soft_current;		//软电流(由 跟踪器/重构器/插补器/硬运算 得到)
	bool			soft_disable;		//软失能
	bool			soft_brake;			//软刹车
	bool			soft_new_curve;	//新软目标曲线
	//输出
	int32_t		foc_location;		//FOC矢量位置
	int32_t		foc_current;		//FOC矢量大小
	//堵转识别
	uint32_t	stall_time_us;	//堵转计时器
	bool			stall_flag;			//堵转标志
	//过载识别
	uint32_t	overload_time_us;	//过载计时器
	bool			overload_flag;		//过载标志
	//状态
	Motor_State		state;			//统一的电机状态
}Motor_Control_Typedef;

void Control_Cur_To_Electric(int16_t current);
void Control_PID_To_Electric(int32_t _speed);
void Control_PID_Init(void);
void Delay_us(uint16_t time);
void Motor_Control_Callback(void);
void Control_DCE_Init(void);




#endif /*MOTOR_CONTROL_H*/



