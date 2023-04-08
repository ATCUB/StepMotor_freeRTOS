/*MT6816驱动头文件*/

#ifndef MT6816DRIVER_H
#define MT6816DRIVER_H

#include "struct_typedef.h"
#include "main.h"


typedef struct{
	//采集数据
	uint16_t	sample_data;	//SPI读取到的数据
	//输出数据
	uint16_t	angle;				//SPI输出的角度
	uint8_t		no_mag_flag;	//磁铁数据有效标志
	uint8_t		pc_flag;			//奇偶校验位
}MT6816_SPI_Signal_Typedef;

typedef struct{
	uint16_t	angle_data;				  //角度数据
	uint16_t	rectify_angle;		  //校准的角度数据
	uint8_t		rectify_valid;		//校准数据有效标志
}MT6816_Typedef;


#define MT6816_SPI_Get_SPI								(SPI1)
#define MT6816_SPI_Get_HSPI								(hspi1)




void REIN_MT6816_Init(void);
void REIN_MT6816_Get_AngleData(void);





















#endif /*MT6816DRIVER_H*/
