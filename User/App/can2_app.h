#ifndef CAN2_APP_H
#define CAN2_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "can1_app.h"

#define CAN_2 hcan2

typedef enum
{	
	
	CAN2_CONNECT_RC_CTRL_STD_ID = 0x1FE,
	// CAN2_CONNECT_INFO_STD_ID = 0x201,
	// CAN2_CONNECT_CM_ENCODE_STD_ID = 0x203,	// C板已经改为底盘拨轮电机ID
	// CAN2_THUMBWHEEL_MOTOR_STD_ID = 0x201,	//	底盘拨轮
	
	CAN2_CONNECT_CHECK_STD_ID = 0x203,
	
	CAN2_YAW_MOTOR1_STD_ID = 0x205,
	CAN2_YAW_MOTOR2_STD_ID = 0x206,	// yaw电机2
	// CAN2_PITCH_MOTOR_STD_ID = 0x207,	//pitch电机
	
	CAN2_CONNECT_GYRO_STD_ID = 0x209,//0x208,
	
	CAN2_CHASSIS_PID_ROTATE_STD_ID =0x20A,// 0X209,//209
	CAN2_CHASSIS_PID_CM_STD_ID = 0X20B,//0X20A,//20A
	CAN2_SHOOT_42mm_ID =0x020C,// 0x020B,         //17mm发射机构裁判信息
	CAN2_SHOOT_JUDGE_ID = 0X020D,//0x020C,        //发射机构裁判信息
	CAN2_CONNECT_CM_SPEED_STD_ID = 0x20E,
} can2_msg_id_e;

void send_shoot_42mm_data(void);
void send_shoot_judge_data(void);
motor_msg_t *get_yaw_motor1_msg_point(void);
motor_msg_t *get_yaw_motor2_msg_point(void);
void can2_message_progress(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);

#endif
