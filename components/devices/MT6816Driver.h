/*MT6816����ͷ�ļ�*/

#ifndef MT6816DRIVER_H
#define MT6816DRIVER_H

#include "struct_typedef.h"
#include "main.h"


typedef struct{
	//�ɼ�����
	uint16_t	sample_data;	//SPI��ȡ��������
	//�������
	uint16_t	angle;				//SPI����ĽǶ�
	uint8_t		no_mag_flag;	//����������Ч��־
	uint8_t		pc_flag;			//��żУ��λ
}MT6816_SPI_Signal_Typedef;

typedef struct{
	uint16_t	angle_data;				  //�Ƕ�����
	uint16_t	rectify_angle;		  //У׼�ĽǶ�����
	uint8_t		rectify_valid;		//У׼������Ч��־
}MT6816_Typedef;


#define MT6816_SPI_Get_SPI								(SPI1)
#define MT6816_SPI_Get_HSPI								(hspi1)




void REIN_MT6816_Init(void);
void REIN_MT6816_Get_AngleData(void);





















#endif /*MT6816DRIVER_H*/
