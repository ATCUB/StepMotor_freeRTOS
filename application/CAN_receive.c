
#include "CAN_receive.h"
#include "main.h"
#include "motor_control.h"


extern CAN_HandleTypeDef hcan;
extern Motor_Control_Typedef 	motor_control;

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

static motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_MAIN_ID:
        {
						static uint8_t i = 0;
						i = CAN_M2_ID - CAN_M1_ID;
						if(rx_data[i] & 0x4)//判断使能
						{
							if(rx_data[i] >> 3) //判断正反转
							motor_control.goal_location += (12800*(rx_data[i] & 0x3)); //计算旋转角度
							else motor_control.goal_location -= (12800*(rx_data[i] & 0x3));
							motor_control.mode_run = Motor_Mode_Digital_Location; //电机使能
						}
						else motor_control.mode_run = Control_Mode_Stop;  //电机刹车
						
            break;
        }
//				case REST_ID_CALL:
//        {
//						motor_control.soft_disable = true;
//						if(motor_control.est_speed >= 6400)break; //速度大于1/8圈
//							
//            break;
//        }

        default:
        {
            break;
        }
    }
}




void CAN_cmd_feedback(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_M4_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor_control.real_location;
    chassis_can_send_data[1] = motor_control.real_location >> 8;
    chassis_can_send_data[2] = motor_control.real_location >> 16;
    chassis_can_send_data[3] = motor_control.real_location >> 24;
    chassis_can_send_data[4] = motor_control.real_lap_location;
    chassis_can_send_data[5] = motor_control.real_lap_location >> 8;
    chassis_can_send_data[6] = motor_control.state;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
