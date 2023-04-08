#ifndef ENCODER_CALI_H
#define ENCODER_CALI_H

#include "main.h"

#define Current_Rated_Current		(3000)		//额定电流(mA)
#define Current_Cali_Current		(1500)		//校准电流(mA)

#define Move_Step_NUM				((int32_t)(200))																//(使用的电机单圈步数)(每步磁场旋转90°)
#define Move_Divide_NUM			((int32_t)(256))																//(每步柔性件控制细分量)
#define Move_Pulse_NUM			((int32_t)(Move_Step_NUM * Move_Divide_NUM))		//(电机单圈脉冲数)

#define CALI_Encode_Bit					((int32_t)(14))																//(编码器位宽)(14位输出精度)
#define CALI_Encode_Res					((int32_t)((0x00000001U) << CALI_Encode_Bit))	//(编码器分辨率)(2^14 = 16384)(16k分辨率)(占用32k校准空间)
#define CALI_Gather_Encode_Res	((int32_t)(CALI_Encode_Res / Move_Step_NUM))	//(校准每采集步编码器分辨率)
typedef enum{
	//无错误
	CALI_No_Error = 0x00,						//数据无错误
	//原始数据出错
	CALI_Error_Average_Dir,					//平均值方向错误
	CALI_Error_Average_Continuity,	//平均值连续性错误
	CALI_Error_PhaseStep,						//阶跃次数错误
	//解析数据出错
	CALI_Error_Analysis_Quantity,		//解析数据量错误
}CALI_Error;

//校准状态
typedef enum{
	CALI_Disable = 0x00,						//失能
	CALI_Forward_Encoder_AutoCali,	//编码器正转自动校准
	CALI_Forward_Measure,						//正向测量
	CALI_Reverse_Ret,								//反向回退
	CALI_Reverse_Gap,								//反向消差
	CALI_Reverse_Measure,						//反向测量
	CALI_Operation,									//解算
}CALI_State;

typedef struct{
	//信号
	bool				trigger;			//触发校准
	CALI_Error	error_code;		//校准错误代码
	uint32_t		error_data;		//校准错误数据
	//读取过程
	CALI_State	state;					//校准状态
	uint32_t		out_location;		//输出位置
	#define			Gather_Quantity	16	//每个采集点采集数量
	uint16_t		gather_count;												//采集计数
	uint16_t		coder_data_gather[Gather_Quantity];	//采集点每次数据
	uint16_t 		coder_data_f[Move_Step_NUM+1];	//校准读取的数据
	uint16_t 		coder_data_r[Move_Step_NUM+1];	//校准读取的数据
	bool				dir;		//校准数据方向
	//全段域校准过程数据
	int32_t		rcd_x, rcd_y;			//寻找区间下标及阶跃差值
	uint32_t	result_num;				//统计数量
}Encode_Cali_Typedef;



void Calibration_Init(void);								//校准器初始化
void Calibration_Interrupt_Callback(void);	//校准器中断回调(稳定中断调用)
void Calibration_Loop_Callback(void);				//校准器主程序回调(非中断调用)





#endif
