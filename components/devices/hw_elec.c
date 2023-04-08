

//Oneself
#include "hw_elec.h"
#include "sin_form.h"
#include <math.h>
#include <stdlib.h>

//GPIO输出
#define Out_AP_H()		(HW_ELEC_AP_GPIO_Port -> BSRR = HW_ELEC_AP_Pin)
#define Out_AP_L()		(HW_ELEC_AP_GPIO_Port -> BRR  = HW_ELEC_AP_Pin)
#define Out_AM_H()		(HW_ELEC_AM_GPIO_Port -> BSRR = HW_ELEC_AM_Pin)
#define Out_AM_L()		(HW_ELEC_AM_GPIO_Port -> BRR  = HW_ELEC_AM_Pin)
#define Out_BP_H()		(HW_ELEC_BP_GPIO_Port -> BSRR = HW_ELEC_BP_Pin)
#define Out_BP_L()		(HW_ELEC_BP_GPIO_Port -> BRR  = HW_ELEC_BP_Pin)
#define Out_BM_H()		(HW_ELEC_BM_GPIO_Port -> BSRR = HW_ELEC_BM_Pin)
#define Out_BM_L()		(HW_ELEC_BM_GPIO_Port -> BRR  = HW_ELEC_BM_Pin)
//TIM输出
#define Out_PWMtoDAC_A(value)		(__HAL_TIM_SET_COMPARE(&HW_ELEC_PWM_Get_HTIM, HW_ELEC_APWM_CHANNEL, value))
#define Out_PWMtoDAC_B(value)		(__HAL_TIM_SET_COMPARE(&HW_ELEC_PWM_Get_HTIM, HW_ELEC_BPWM_CHANNEL, value))


//硬件电流实例
Coil_Typedef		coil_a;	//电流控制
Coil_Typedef		coil_b;	//电流控制

/**
  * @brief  12位基准电压混合输出
  * @param  elec_va: 电流通道a基准
  * @param  elec_vb: 电流通道b基准
  * @retval NULL
  */
void CurrentControl_VREF_12Bit_MixOut(uint16_t elec_va, uint16_t elec_vb)
{
	Out_PWMtoDAC_A((elec_va >> 2));
	Out_PWMtoDAC_B((elec_vb >> 2));
}



/**
  * @brief  设置睡眠
  * @param  NULL
  * @retval NULL
  */
void REIN_HW_Elec_SetSleep(void)
{
	coil_a.dac_reg = 0;
	coil_b.dac_reg = 0;
	CurrentControl_VREF_12Bit_MixOut(coil_a.dac_reg, coil_b.dac_reg);
	Out_AP_L();	Out_AM_L();
	Out_BP_L();	Out_BM_L();
}

/**
  * @brief  设置驱动刹车
  * @param  NULL
  * @retval NULL
  */
void REIN_HW_Elec_SetBrake(void)
{
	coil_a.dac_reg = 0;
	coil_b.dac_reg = 0;
	CurrentControl_VREF_12Bit_MixOut(coil_a.dac_reg, coil_b.dac_reg);
	Out_AP_H();	Out_AM_H();
	Out_BP_H();	Out_BM_H();
}

/**
  * @brief  设置输出细分电流
  * @param  divide:  细分 (0 ~ 细分数)
  * @param  dac_reg: 电流 (0 ~ 3300mA)
  * @retval NULL
  */
void REIN_HW_Elec_SetDivideElec(uint32_t divide, int32_t elec_ma)
{
	//由细分数获得数组指针
	coil_b.conver = (divide) & (0x000003FF);								//对1024取余
	coil_a.conver = (coil_b.conver + (256)) & (0x000003FF);	//对1024取余
	
	//由数据指针获得整形数据(空间换时间方案)
	coil_a.sin_data = sin_pi_m2[coil_a.conver];
	coil_b.sin_data = sin_pi_m2[coil_b.conver];

	//由整形数据获得DAC寄存器数据
	uint32_t dac_reg = abs(elec_ma);								//电压电流关系为1:1(检流电阻为0.1欧)
	dac_reg = (uint32_t)(dac_reg * 5083) >> 12; 		//(dac_reg * 4095 / 3300)的变种
	dac_reg = dac_reg & (0x00000FFF);								//(对4096取余)(向小取整)(舍弃符号位)
	coil_a.dac_reg = (uint32_t)(dac_reg * abs(coil_a.sin_data)) >> sin_pi_m2_dpiybit;	//(--- / sin_1024_dpiy)的变种
	coil_b.dac_reg = (uint32_t)(dac_reg * abs(coil_b.sin_data)) >> sin_pi_m2_dpiybit;	//(--- / sin_1024_dpiy)的变种

	//DAC输出
	CurrentControl_VREF_12Bit_MixOut(coil_a.dac_reg, coil_b.dac_reg);
	if(coil_a.sin_data > 0)				{	Out_AP_H();		Out_AM_L();	}
	else if(coil_a.sin_data < 0)	{	Out_AP_L();		Out_AM_H();	}
	else 													{	Out_AP_H();		Out_AM_H();	}
	if(coil_b.sin_data > 0)				{	Out_BP_H();		Out_BM_L();	}
	else if(coil_b.sin_data < 0)	{	Out_BP_L();		Out_BM_H();	}
	else													{	Out_BP_H();		Out_BM_H();	}
}
