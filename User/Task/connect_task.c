#include "connect_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "can1_app.h"
#include "can2_app.h"
#include "main.h"
#include "chassis_task.h"
#include "rule.h"



extern CAN_RxHeaderTypeDef can2_rx_header;
extern uint8_t can2_rx_data[CAN_RX_BUF_SIZE];
extern CAN_TxHeaderTypeDef can2_tx_header;
extern uint8_t can2_tx_data[CAN_TX_BUF_SIZE];
extern chassis_control_data_t chassis_control_data;

connect_t connect_data = 
{
	.ONE_CHECK[0] = 'o',
	.ONE_CHECK[1] = 'j',
	.ONE_CHECK[2] = 'b',
	.ONE_CHECK[3] = 'k',
	.ONE_CHECK[4] = '?',
	.ONE_CHECK[5] = 0,
	.ONE_CHECK[6] = 0,
	.ONE_CHECK[7] = 0,
	
	.TWO_CHECK[0] = 'd',
	.TWO_CHECK[1] = 'd',
	.TWO_CHECK[2] = 'i',
	.TWO_CHECK[3] = 'u',
	.TWO_CHECK[4] = '.',
	.TWO_CHECK[5] = 0,
	.TWO_CHECK[6] = 0,
	.TWO_CHECK[7] = 0,	
};

/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note          
  */

void receive_check_package_process(connect_t *connect_data, uint8_t aData[])
{
	if(
		aData[0] == connect_data->ONE_CHECK[0] &&  \
		aData[1] == connect_data->ONE_CHECK[1] &&  \
		aData[2] == connect_data->ONE_CHECK[2] &&  \
		aData[3] == connect_data->ONE_CHECK[3] &&  \
		aData[4] == connect_data->ONE_CHECK[4] &&  \
		aData[5] == connect_data->ONE_CHECK[5] &&  \
		aData[6] == connect_data->ONE_CHECK[6] &&  \
		aData[7] == connect_data->ONE_CHECK[7])
	{
		connect_data->receive_success_flag = 1;
	}
}
/**
  * @brief         底盘接收到数据之后 发送连接成功的数据包给云台
  * @author         
  * @param[in] 
  * @retval	
  * @note          
  */
 

void send_connect_success_package(const uint8_t *check)  
{	
	can2_tx_header.StdId = CAN2_CONNECT_CHECK_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;
    
    can2_tx_data[0] = (uint8_t)check[0];
    can2_tx_data[1] = (uint8_t)check[1];
    can2_tx_data[2] = (uint8_t)check[2];
    can2_tx_data[3] = (uint8_t)check[3];
    can2_tx_data[4] = (uint8_t)check[4];
    can2_tx_data[5] = (uint8_t)check[5];
    can2_tx_data[6] = (uint8_t)check[6];
    can2_tx_data[7] = (uint8_t)check[7];
    HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0  );
}

void check_connect(connect_t *connect_data)
{
	connect_data->receive_rc_data_flag = 0;
	connect_data->receive_success_flag = 0;
	while(!connect_data->receive_success_flag);//等待接收one_check数据包
	do
	{
		send_connect_success_package(connect_data->TWO_CHECK);//发送two_check数据包
		HAL_Delay(10);
		//osDelay(10);
	}while(!connect_data->receive_rc_data_flag);//等待接收到rc数据，停止发送two_check
	
	connect_data->receive_success_flag = 0;
	
}
/**
  * @brief          can2接收rc数据保存
  * @author         
  * @param[in] 
  * @retval	
  * @note        
  */
void connect_rc_ctrl_process(connect_t *connect_data, uint8_t aData[])
{
	chassis_control_data.magazine_control_flag = ((aData[0]&0x80)>>7);
	chassis_control_data.thumbwheel_move_flag = ((aData[0]&0x40)>>6);
	chassis_control_data.thumbwheel_contrary_flag= ((aData[0]&0x20)>>5);
	chassis_control_data.thumbwheel_remote_flag=((aData[0]&0x10)>>4);
	connect_data->can2_rc_ctrl.control_mode = (aData[0]&0x03);
	connect_data->can2_rc_ctrl.work_mode = (aData[1]);
	connect_data->can2_rc_ctrl.rc.ch2 = (aData[2]<<8)|aData[3];
	connect_data->can2_rc_ctrl.rc.ch3 = (aData[4]<<8)|aData[5];
	connect_data->can2_rc_ctrl.mouse.key = (aData[6]<<8)|aData[7];
	connect_data->can2_rc_ctrl.rc.ch2 -= RC_CHANNEL_VALUE_MIDDLE;
	connect_data->can2_rc_ctrl.rc.ch3 -= RC_CHANNEL_VALUE_MIDDLE;
	
	connect_data->receive_rc_data_flag = 1;//表示已经接收到了can2的rc数据
}
void connect_gyro_data_process(connect_t *connect_data, uint8_t aData[])
{
	if(aData[6]==0)
	{
		connect_data->can2_rc_ctrl.gyro.yaw_set = ((float)(((aData[0]<<16) | (aData[1]<<8) | aData[2]))/10);
	}
	else
	{
		connect_data->can2_rc_ctrl.gyro.yaw_set = -(((float)(((aData[0]<<16) | (aData[1]<<8) | aData[2]))/10));
	}
	if(aData[7]==0)
	{
		connect_data->can2_rc_ctrl.gyro.yaw_fdb = ((float)(((aData[3]<<16) | (aData[4]<<8) | aData[5]))/10);
	}
	else
	{
		connect_data->can2_rc_ctrl.gyro.yaw_fdb = -(((float)(((aData[3]<<16) | (aData[4]<<8) | aData[5]))/10));
	}
	connect_data->receive_rc_data_flag = 1;//表示已经接收到了can2的rc数据

}

extern chassis_pid_t chassis_pid;
void chassis_pid_process(pid_t *pid, uint8_t aData[], connect_t *connect_data)
{
	pid->kp = ((float)((aData[0]<<8)|aData[1]))/10;
	pid->ki = ((float)((aData[2]<<8)|aData[3]))/10;
	pid->kd = ((float)((aData[4]<<8)|aData[5]))/10;
	connect_data->receive_rc_data_flag = 1;//表示已经接收到了can2的rc数据
	
}
/**
  * @brief          连接初始化
  * @author         
  * @param[in] 
  * @retval	
  * @note           考虑未连接 挂起任务调度器 或进入临界区 while(1)
  */
// void send_cm_encode_package(connect_t *connect)
// {
// 	can2_tx_header.StdId = CAN2_CONNECT_CM_ENCODE_STD_ID;
//     can2_tx_header.IDE = CAN_ID_STD;
//     can2_tx_header.RTR = CAN_RTR_DATA;
//     can2_tx_header.DLC = 0x08;
    
//     can2_tx_data[0] = (uint8_t)(connect->cm1_msg->encoder.raw_value >> 8);
//     can2_tx_data[1] = (uint8_t)(connect->cm1_msg->encoder.raw_value);
//     can2_tx_data[2] = (uint8_t)(connect->cm2_msg->encoder.raw_value >> 8);
//     can2_tx_data[3] = (uint8_t)(connect->cm2_msg->encoder.raw_value);
//     can2_tx_data[4] = (uint8_t)(connect->cm3_msg->encoder.raw_value >> 8);
//     can2_tx_data[5] = (uint8_t)(connect->cm3_msg->encoder.raw_value);
//     can2_tx_data[6] = (uint8_t)(connect->cm4_msg->encoder.raw_value >> 8);
//     can2_tx_data[7] = (uint8_t)(connect->cm4_msg->encoder.raw_value);
// 	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0  );
// }
void send_cm_speed_package(connect_t *connect)
{
	can2_tx_header.StdId = CAN2_CONNECT_CM_SPEED_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;
    
    can2_tx_data[0] = (uint8_t)(connect->cm1_msg->encoder.filter_rate >> 8);
    can2_tx_data[1] = (uint8_t)(connect->cm1_msg->encoder.filter_rate);
    can2_tx_data[2] = (uint8_t)(connect->cm2_msg->encoder.filter_rate >> 8);
    can2_tx_data[3] = (uint8_t)(connect->cm2_msg->encoder.filter_rate);
    can2_tx_data[4] = (uint8_t)(connect->cm3_msg->encoder.filter_rate >> 8);
    can2_tx_data[5] = (uint8_t)(connect->cm3_msg->encoder.filter_rate);
    can2_tx_data[6] = (uint8_t)(connect->cm4_msg->encoder.filter_rate >> 8);
    can2_tx_data[7] = (uint8_t)(connect->cm4_msg->encoder.filter_rate);
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0  );
}

void send_cm_to_gimbal(connect_t *connect)
{
	// if(connect->can2_rc_ctrl.control_mode == GUI_CALI_MODE)
	// {
	// 	send_cm_encode_package(connect);//底盘电机编码信息
	// }
	send_cm_speed_package(connect);//底盘电机速度信息
}
/**
  * @brief          连接初始化
  * @author         
  * @param[in] 
  * @retval	
  * @note           考虑未连接 挂起任务调度器 或进入临界区 while(1)
  */
void connect_init(connect_t *connect_data) 
{
	connect_data->cm1_msg = get_cm1_msg_point();
	connect_data->cm2_msg = get_cm2_msg_point();
	connect_data->cm3_msg = get_cm3_msg_point();
	connect_data->cm4_msg = get_cm4_msg_point();
	
//	check_connect(connect_data);
}

/**
  * @brief          连接任务
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  */
 
void connect_task(void *argument)
{
	connect_init(&connect_data);	//返回4个电机数据（motor_msg_t里面变量）参数指针
	while(1)
	{
		// send_cm_to_gimbal(&connect_data);
		send_shoot_42mm_data();	///发送枪管数据，冷却信息、温度等
		send_shoot_judge_data();	///发送子弹射击速度、伤害值、以及机器人编号等
		vTaskDelay(15);
	}
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  */
connect_t *get_connect_data_point(void)
{
	return &connect_data;
}
