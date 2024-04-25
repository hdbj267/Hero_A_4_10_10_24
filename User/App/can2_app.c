
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "connect_task.h"
#include "can.h"
#include "can1_app.h"
#include "can2_app.h"
#include "judge.h"
#include "stm32f4xx_hal_can.h"
#include "chassis_task.h"

extern CAN_RxHeaderTypeDef can2_rx_header;
extern uint8_t can2_rx_data[CAN_RX_BUF_SIZE];
extern CAN_TxHeaderTypeDef can2_tx_header;
extern uint8_t can2_tx_data[CAN_TX_BUF_SIZE];
extern CAN_HandleTypeDef hcan2;

extern chassis_control_data_t chassis_control_data;
motor_msg_t yaw_motor1_msg = {0};
motor_msg_t yaw_motor2_msg = {0};
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    Ĭ�ϵ����can����Ƶ��Ϊ1KHZ       
  */ 

static void yaw_motor_msg_process(motor_msg_t *m, uint8_t aData[])
{
	int16_t i;
	m->encoder.filter_rate_sum = 0;//��������
	m->encoder.last_raw_value = m->encoder.raw_value; 
	if(m->encoder.start_flag==0)//�ϵ�ɼ�ԭʼ�Ƕ�
	{
		m->encoder.ecd_bias = (aData[0]<<8)|aData[1];//��ʼλ��
		m->encoder.last_raw_value = (aData[0]<<8)|aData[1];
		m->encoder.raw_value = m->encoder.last_raw_value;
		m->encoder.start_flag = 1;
	}
	else
	{
		m->encoder.raw_value = (aData[0]<<8)|aData[1];
	}
	
	m->encoder.diff = m->encoder.raw_value - m->encoder.last_raw_value;
	if(m->encoder.diff < -6000)//���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�                         
	{                          //7500���ݿ������ڿɵ�������֤һ������������ ת�ӻ�е�Ƕȱ仯С��8191-7500=691���� 
		m->encoder.round_cnt ++;
		m->encoder.ecd_raw_rate = m->encoder.diff + 8192;
	}
	else if(m->encoder.diff > 6000)
	{
		m->encoder.round_cnt --;
		m->encoder.ecd_raw_rate = m->encoder.diff - 8192;
	}
	else
	{
		m->encoder.ecd_raw_rate = m->encoder.diff;
	}
	//����õ��Ƕ�ֵ����Χ���������
	m->encoder.ecd_angle = (float)(m->encoder.raw_value - m->encoder.ecd_bias)*360/8192  \
								   + m->encoder.round_cnt * 360;
	
	m->encoder.rate_buf[m->encoder.buf_count++] = m->encoder.ecd_raw_rate;
	if(m->encoder.buf_count == RATE_BUF_SIZE)
	{
		m->encoder.buf_count = 0;
	}
	//�����ٶ�ƽ��ֵ
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		m->encoder.filter_rate_sum += m->encoder.rate_buf[i];
	}
	m->encoder.filter_rate = (int32_t)(m->encoder.filter_rate_sum/RATE_BUF_SIZE);	
	/*---------------------�Ǳ���������------------------------*/
	m->speed_rpm = (uint16_t)(aData[2] << 8 | aData[3]);     
	m->given_current = (uint16_t)(aData[4] << 8 | aData[5]); 
	m->temperate = aData[6];          
}

typedef union float_char_
{
	float a;
	uint8_t b[4];
}float_char;
float_char f_c;
extern chassis_pid_t chassis_pid;
void can2_message_progress(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	if(pHeader == NULL || aData == NULL )
	{
		return;
	}
	switch(pHeader->StdId)
	{
		//get rc control 
		// case CAN2_CONNECT_CHECK_STD_ID:
		// {
		// 	receive_check_package_process(&connect_data, aData);
		// }break;
		case CAN2_CONNECT_RC_CTRL_STD_ID:
		{
			connect_rc_ctrl_process(&connect_data ,aData);
		}break;
		case CAN2_CONNECT_GYRO_STD_ID:
		{
			connect_gyro_data_process(&connect_data ,aData);
		}break;
		case CAN2_CHASSIS_PID_ROTATE_STD_ID:
		{
			chassis_pid_process(&chassis_pid.rotate_pid, aData, &connect_data);
		}break;
		case CAN2_CHASSIS_PID_CM_STD_ID:
		{
			chassis_pid_process(&chassis_pid.cm1_pid, aData, &connect_data);
			chassis_pid_process(&chassis_pid.cm2_pid, aData, &connect_data);
			chassis_pid_process(&chassis_pid.cm3_pid, aData, &connect_data);
			chassis_pid_process(&chassis_pid.cm4_pid, aData, &connect_data);
		}break;
		// case CAN2_CONNECT_INFO_STD_ID:
		// {
			
		// }break;
		//get gimbal control 
		case CAN2_YAW_MOTOR1_STD_ID:
		{
			yaw_motor_msg_process(&yaw_motor1_msg ,aData); 
		}break;
		case CAN2_YAW_MOTOR2_STD_ID:
		{
			yaw_motor_msg_process(&yaw_motor2_msg ,aData); 
		}break;


		default: break;
	}
}


extern ext_game_robot_status_t robot_status;
extern ext_power_heat_data_t power_heat;
extern ext_shoot_data_t shoot_data; 
extern ext_robot_hurt_t robot_hurt;

void send_shoot_42mm_data(void)  
{
	can2_tx_header.StdId = CAN2_SHOOT_42mm_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;

    can2_tx_data[0] = (uint8_t)(robot_status.shooter_id1_42mm_cooling_rate>>8);	
    can2_tx_data[1] = (uint8_t)(robot_status.shooter_id1_42mm_cooling_rate);
    can2_tx_data[2] = (uint8_t)(robot_status.shooter_id1_42mm_cooling_limit>>8);
    can2_tx_data[3] = (uint8_t)(robot_status.shooter_id1_42mm_cooling_limit);
	can2_tx_data[4] = (uint8_t)(robot_status.shooter_id1_42mm_speed_limit>>8);
    can2_tx_data[5] = (uint8_t)(robot_status.shooter_id1_42mm_speed_limit);
    can2_tx_data[6] = (uint8_t)(power_heat.shooter_id1_42mm_cooling_heat>>8);
    can2_tx_data[7] = (uint8_t)(power_heat.shooter_id1_42mm_cooling_heat);
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
}
void send_shoot_judge_data(void)  
{
	can2_tx_header.StdId = CAN2_SHOOT_JUDGE_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;

    can2_tx_data[0] = (uint8_t)(((uint16_t)(shoot_data.bullet_speed*100))>>8);     //������λС���� 
    can2_tx_data[1] = (uint8_t)((uint16_t)(shoot_data.bullet_speed*100));
    can2_tx_data[2] = (uint8_t)(robot_hurt.hurt_type);
    can2_tx_data[3] = (uint8_t)(robot_status.mains_power_shooter_output);
	can2_tx_data[4] = (uint8_t)(robot_status.robot_id);
    can2_tx_data[5] = 0;
    can2_tx_data[6] = 0;
    can2_tx_data[7] = 0;
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
}

motor_msg_t *get_yaw_motor1_msg_point(void)
{
	return &yaw_motor1_msg;
}

motor_msg_t *get_yaw_motor2_msg_point(void)
{
	return &yaw_motor2_msg;
}

