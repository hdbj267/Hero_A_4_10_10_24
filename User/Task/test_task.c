/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HuangYe
 * @Teammate：
 * @Version: V3.0
 * @Date:2020.3.10
 * @Description:           
 * @Note:      
 * @Others: 
**/
#include "test_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "judge.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "usart.h"
extern ext_game_robot_status_t robot_status;
/**
  * @brief  通过自己的ID，返回对应操作手客户端ID
  * @param  void
  * @retval 
  * @attention  
  */
int Operator_ID(void)
{
	uint8_t Operator_id;
	if(robot_status.robot_id < 10)        //红方
	{
		Operator_id = robot_status.robot_id + 0x0100 ;
	}
	else if (robot_status.robot_id > 10)  //蓝方
	{
		Operator_id = robot_status.robot_id + 0x0064 ;
	}
	return Operator_id;
}
/**
  * @brief  在客户端显示浮点数
  * @param  void
  * @retval 
  * @attention  
  */
void Display_float(graphic_data_struct_t *configure, float num)             
{
	configure->graphic_name[0] = 0;
	configure->operate_tpye = 1;
	configure->graphic_tpye = 5;
	configure->layer = 1;
	configure->color = 7;
	
	configure->start_angle = 5;
	configure->end_angle = 2;
	configure->width = 3;
	configure->start_x = 1;
	configure->start_y = 1;
	// configure->radius = num >>22;
	// configure->end_x = num >>11;
	configure->end_y = num;
}

/**
  * @brief  在客户端显示整形数
  * @param  void
  * @retval 
  * @attention  
  */
void Display_int(graphic_data_struct_t *configure, int32_t num)
{
	configure->graphic_name[0] = 1;
	configure->operate_tpye = 1;
	configure->graphic_tpye = 6;
	configure->layer = 1;
	configure->color = 7;

	configure->start_angle = 5;
	configure->end_angle = NULL;
	configure->width = 3;
	configure->start_x = 1;
	configure->start_y = 1;
	configure->radius = num >>22;
	configure->end_x = num >>11;
	configure->end_y = num;
}

/**
  * @brief  在客户端显示字符
  * @param  void
  * @retval 
  * @attention  
  */
void Display_char(void)
{
	;
}

/**
  * @brief  上传自定义数据
  * @param  void
  * @retval void    
  * @attention    数据打包,打包完成后通过串口发送到裁判系统       *hyj
  *          此函数可将数据传送到操作手界面显示（未调用，开学调试时，看需求！）
  */
uint8_t CliendTxBuffer[SEND_MAX_LEN];
ext_Send_User_Data_t      ShowData;			//客户端信息
void Show_User_Data(void)
{	
	ShowData.txFrameHeader.SOF = 0xA5;
	ShowData.txFrameHeader.data_length = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_graphic_single_t);       //client_custom_data_t
	ShowData.txFrameHeader.seq = 0;
	memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(frame_header_t));//写入帧头数据
	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(frame_header_t));//写入帧头CRC8校验码
	
	ShowData.CmdID = ROBOT_COMMUNICATION_ID;
	
	ShowData.dataFrameHeader.data_cmd_id = 0x0101;//数据段内容的ID
	ShowData.dataFrameHeader.send_ID 	 = robot_status.robot_id ;//发送者的ID
	ShowData.dataFrameHeader.receiver_ID = Operator_ID();//客户端的ID，只能为发送者机器人对应的客户端
	
	/*- 自定义内容 -******暂时的，开学调试看需要什么 *hyj */
	// ShowData.clientData.data1 = 11.11;
	// ShowData.clientData.data2 = 22.22;
	// ShowData.clientData.data3 = 33.33;
	// ShowData.clientData.data4 = 44;
	Display_int(&ShowData.clientData.grapic_data_struct,111);
	/*--------------*/
	//打包写入数据段
	memcpy(	
			CliendTxBuffer + 5, 
			(uint8_t*)&ShowData.CmdID, 
			(sizeof(ShowData.CmdID)+ sizeof(ShowData.dataFrameHeader)+ sizeof(ShowData.clientData))
		  );			
			
	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(ShowData));//写入数据段CRC16帧尾校验码	

	// HAL_UART_Transmit_IT(&huart3 ,(uint8_t*)CliendTxBuffer,sizeof(CliendTxBuffer));
	HAL_UART_Transmit(&huart3, CliendTxBuffer, sizeof(CliendTxBuffer), 1000);      //发数据  
}


void test_task(void *argument)
{
    while(1)
    {		
		Show_User_Data();		///数据打包,打包完成后通过串口发送到裁判系统,此函数可将数据传送到操作手界面显示
		vTaskDelay(TRANSMIT_SHOW_DATA_TIME);       //35ms一次
    }
}



