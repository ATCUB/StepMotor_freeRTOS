



#ifndef HW_ELEC_H
#define HW_ELEC_H

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"

extern TIM_HandleTypeDef htim2;

#define	HW_ELEC_PWM_Get_TIM							(TIM2)
#define	HW_ELEC_PWM_Get_HTIM						(htim2)
#define HW_ELEC_APWM_CHANNEL						(TIM_CHANNEL_4)
#define HW_ELEC_BPWM_CHANNEL						(TIM_CHANNEL_3)
/**
  * 线圈定义
**/
typedef struct{
	uint16_t conver;		//SIN数组指针
	int16_t  sin_data;	//SIN换算数值
	uint16_t dac_reg;		//12位DAC数值
}Coil_Typedef;

extern Coil_Typedef		coil_a;	//电流控制
extern Coil_Typedef		coil_b;	//电流控制

void REIN_HW_Elec_Init(void);																				//硬件电流控制初始化
void REIN_HW_Elec_SetSleep(void);																		//硬件电流设置驱动睡眠
void REIN_HW_Elec_SetBrake(void);																		//硬件电流设置驱动刹车
void REIN_HW_Elec_SetDivideElec(uint32_t divide, int32_t elec_ma);	//硬件电流设置输出细分电流

#ifdef __cplusplus
}
#endif

#endif

