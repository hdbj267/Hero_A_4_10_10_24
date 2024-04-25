/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HuangYe
 * @Teammate��
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
  * @brief  ͨ���Լ���ID�����ض�Ӧ�����ֿͻ���ID
  * @param  void
  * @retval 
  * @attention  
  */
int Operator_ID(void)
{
	uint8_t Operator_id;
	if(robot_status.robot_id < 10)        //�췽
	{
		Operator_id = robot_status.robot_id + 0x0100 ;
	}
	else if (robot_status.robot_id > 10)  //����
	{
		Operator_id = robot_status.robot_id + 0x0064 ;
	}
	return Operator_id;
}
/**
  * @brief  �ڿͻ�����ʾ������
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
  * @brief  �ڿͻ�����ʾ������
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
  * @brief  �ڿͻ�����ʾ�ַ�
  * @param  void
  * @retval 
  * @attention  
  */
void Display_char(void)
{
	;
}

/**
  * @brief  �ϴ��Զ�������
  * @param  void
  * @retval void    
  * @attention    ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ       *hyj
  *          �˺����ɽ����ݴ��͵������ֽ�����ʾ��δ���ã���ѧ����ʱ�������󣡣�
  */
uint8_t CliendTxBuffer[SEND_MAX_LEN];
ext_Send_User_Data_t      ShowData;			//�ͻ�����Ϣ
void Show_User_Data(void)
{	
	ShowData.txFrameHeader.SOF = 0xA5;
	ShowData.txFrameHeader.data_length = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_graphic_single_t);       //client_custom_data_t
	ShowData.txFrameHeader.seq = 0;
	memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(frame_header_t));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(frame_header_t));//д��֡ͷCRC8У����
	
	ShowData.CmdID = ROBOT_COMMUNICATION_ID;
	
	ShowData.dataFrameHeader.data_cmd_id = 0x0101;//���ݶ����ݵ�ID
	ShowData.dataFrameHeader.send_ID 	 = robot_status.robot_id ;//�����ߵ�ID
	ShowData.dataFrameHeader.receiver_ID = Operator_ID();//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	
	/*- �Զ������� -******��ʱ�ģ���ѧ���Կ���Ҫʲô *hyj */
	// ShowData.clientData.data1 = 11.11;
	// ShowData.clientData.data2 = 22.22;
	// ShowData.clientData.data3 = 33.33;
	// ShowData.clientData.data4 = 44;
	Display_int(&ShowData.clientData.grapic_data_struct,111);
	/*--------------*/
	//���д�����ݶ�
	memcpy(	
			CliendTxBuffer + 5, 
			(uint8_t*)&ShowData.CmdID, 
			(sizeof(ShowData.CmdID)+ sizeof(ShowData.dataFrameHeader)+ sizeof(ShowData.clientData))
		  );			
			
	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(ShowData));//д�����ݶ�CRC16֡βУ����	

	// HAL_UART_Transmit_IT(&huart3 ,(uint8_t*)CliendTxBuffer,sizeof(CliendTxBuffer));
	HAL_UART_Transmit(&huart3, CliendTxBuffer, sizeof(CliendTxBuffer), 1000);      //������  
}


void test_task(void *argument)
{
    while(1)
    {		
		Show_User_Data();		///���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ,�˺����ɽ����ݴ��͵������ֽ�����ʾ
		vTaskDelay(TRANSMIT_SHOW_DATA_TIME);       //35msһ��
    }
}



