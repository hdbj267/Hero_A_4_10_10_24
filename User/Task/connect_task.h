#ifndef CONNECT_TASK_H
#define CONNECT_TASK_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "can1_app.h"
#include "pid.h"

#define RC_CHANNEL_VALUE_MAX            (1684u)      
#define RC_CHANNEL_VALUE_MIDDLE         (1024u) 
#define RC_CHANNEL_VALUE_MIN            (364u) 

typedef struct   //can2�����rc����
{
	uint8_t control_mode;
	uint8_t work_mode;
	struct 
	{
		int16_t ch2;
		int16_t ch3;
	}rc;
	struct 
	{
		float yaw_set;
		float yaw_fdb;
	}gyro;
	struct
	{
		int16_t key;
	}mouse;

}can2_rc_ctrl_t;

//typedef struct //can2�������Ϣ����
//{

//}can2_info_t;

typedef struct
{
	//RC_ctrl_t *rc_ctrl;
	motor_msg_t *cm1_msg;
	motor_msg_t *cm2_msg;
	motor_msg_t *cm3_msg;
	motor_msg_t *cm4_msg;
	
	can2_rc_ctrl_t can2_rc_ctrl;
	const uint8_t ONE_CHECK[8];//����������ݰ�
	const uint8_t TWO_CHECK[8];
	uint8_t receive_success_flag;
	uint8_t receive_rc_data_flag;
}connect_t;


extern connect_t connect_data;
void receive_check_package_process(connect_t *connect_data, uint8_t aData[]);
void connect_rc_ctrl_process(connect_t *connect_data, uint8_t aData[]);
void connect_gyro_data_process(connect_t *connect_data, uint8_t aData[]);
void chassis_pid_process(pid_t *pid, uint8_t aData[], connect_t *connect_data);

void get_connect_data(CAN_HandleTypeDef * msg);
void check_connect(connect_t *connect_data);
connect_t *get_connect_data_point(void);
void connect_task(void *argument);
#endif
