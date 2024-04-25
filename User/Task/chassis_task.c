#include "chassis_task.h"
#include "pid.h"
#include "can1_app.h"
#include "can2_app.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "pid.h"
#include "math.h"
#include "rule.h"
#include "connect_task.h"
#include "main.h"
#include "oled.h"
#include "stdio.h"
#include "stdlib.h"
#include "time.h"

chassis_control_data_t chassis_control_data;
chassis_pid_t chassis_pid;
float last_in=0;
float T=0.001;
float out;

float forwardfeed(float in,chassis_control_data_t *chassis)
{
//	int k1=2000000000,k2=491200000;
// out=k1*(in-last_in)/T-k2*in;
//	if((in<70)&&(in>-70))
//		out=0;
//	else
//	out=0.7*(0.05*in-79.36);
	//last_in=in;
	if(in<17)
		out=0;
	else if(in>=17&&in<103)
		out=(double)0.0778*in+562.3;
	else if(in>=103&&in<157)
		out=(double)0.1319*in+559.8;
	else if(in>=157&&in<193)
		out=(double)0.0804*in+567.7;
	else if(in>=193&&in<232)
		out=(double)0.0553*in+572.5;
	else if(in>=232&&in<275)
		out=(double)0.0602*in+571.5;
	else if(in>=275)
		out=(double)0.0399*in+575.5;
if(chassis->connect->can2_rc_ctrl.control_mode==GUI_CALI_MODE)
	out = 0;
	return out;
}


cali_chassis_t cali_chassis_pid =
{
/**
* @brief ����PID����        
* @note    
*/
	.cm_pid.kp = 20.0,  //6.2  //10stable  20
	.cm_pid.ki = 0.50,  //0.3   //0
	.cm_pid.kd = 200.0,
	.cm_pid.ioutput_max = 1000,
	.cm_pid.output_max = 20000,
	.cm_pid.mode = PID_POSITION,			//PID_DELTA	PID_POSITION
/**
* @brief ��תPID����        
* @note    
*/	
	.rotate_pid.kp =0.8,// 3.2,		1.8
	.rotate_pid.ki = 0,
	.rotate_pid.kd =0,// 300, 180
	.rotate_pid.ioutput_max = 1000,
	.rotate_pid.output_max = 5000,
	.rotate_pid.mode = PID_POSITION,	//PID_POSITION	//������ȷ�� Ŀǰ�����ܴ�
/**
* @brief ����PID����       
* @note    
*/
	.thumbwheel_pid.kp = 50.0,
	.thumbwheel_pid.ki = 0.0,
	.thumbwheel_pid.kd = 0.0,
	.thumbwheel_pid.ioutput_max = 1000,
	.thumbwheel_pid.output_max = 8000,//��̫�󿨵���������
	.thumbwheel_pid.mode = PID_POSITION,	
};
/**
* @brief ȡ����ֵ����      
* @note    
*/
int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
int32_t gimbal_round = 0;			///��̨ת��
int32_t gimbal_raw_value = 0;		
uint8_t correct_direction_flag = 0;	// ��̨������־��Ĭ��ʹ��Yaw1
uint16_t press_key_x_time = 0;
/**
  * @brief     ����PID��ʼ��   
  * @author         
  * @param[in]      
  * @retval			
  * @note       �Ѻ���У׼��Ĳ�������ǰ��Ľṹ�����    
  */
void chassis_pid_init(pid_t *pid, cali_pid_t *cali_pid)
{
	pid->kp = cali_pid->kp;
	pid->ki = cali_pid->ki;
	pid->kd = cali_pid->kd;
	
	pid->ioutMax = cali_pid->ioutput_max;
	pid->outputMax = cali_pid->output_max;
	
	pid->mode = cali_pid->mode;
	
	pid->Calc = &PID_Calc;
	pid->Reset =  &PID_Reset;
}


/**
  * @brief          ���̳�ʼ��
  * @author         
  * @param[in]      
  * @retval			
  * @note           ���̳�ʼ������ʼ���ĸ���������֡�yaw�ỹ��ң�������ӵ�ָ���Լ������ĸ������С���ݵ��ٶ�pid
  */
void chassis_init(chassis_control_data_t *chassis, chassis_pid_t *chassis_pid)
{
	chassis->connect = get_connect_data_point();
	chassis->cm1_msg = get_cm1_msg_point();
	chassis->cm2_msg = get_cm2_msg_point();
	chassis->cm3_msg = get_cm3_msg_point();
	chassis->cm4_msg = get_cm4_msg_point();
	chassis->thumbwheel_motor_msg = get_thumbwheel_motor_msg_point();
	chassis->yaw_motor1_msg = get_yaw_motor1_msg_point();
	chassis->yaw_motor2_msg = get_yaw_motor2_msg_point();

	chassis_pid_init(&chassis_pid->cm1_pid, &cali_chassis_pid.cm_pid);
	chassis_pid_init(&chassis_pid->cm2_pid, &cali_chassis_pid.cm_pid);
	chassis_pid_init(&chassis_pid->cm3_pid, &cali_chassis_pid.cm_pid);
	chassis_pid_init(&chassis_pid->cm4_pid, &cali_chassis_pid.cm_pid);
	chassis_pid_init(&chassis_pid->rotate_pid, &cali_chassis_pid.rotate_pid);

	chassis_pid_init(&chassis_pid->thumbwheel_pid, \
	&cali_chassis_pid.thumbwheel_pid);
}
/**
  * @brief        С�����µ��˶����� 
  * @author         
  * @param[in]      
  * @retval			
  * @note        //ǰ�����ָ��ţ�����Ϊ6020�Ƿ���װ�ģ�Ҳ�ɸ���ʵ�ʵ��Եõ� 
  */
void rotate_motion_mode_process(chassis_control_data_t *chassis)
{
	chassis->rotate_motion.yaw_current_ecd = gimbal_raw_value;
	
	if (!correct_direction_flag)
	{
		chassis->rotate_motion.yaw_init_ecd = GAMBAL_YAW1_INIT_ENCODE_VALUE;
	}
	else
	{
		chassis->rotate_motion.yaw_init_ecd = GAMBAL_YAW2_INIT_ENCODE_VALUE;
	}
	if( chassis->chassis_control_mode_flag )
	{
		chassis->rotate_motion.yaw_init_ecd = chassis->rotate_motion.yaw_init_ecd - 1488 ;
	}
	//�õ�����ֵ����ʱ�뷽��0��360�ȱ仯�ĽǶ�
	if(chassis->rotate_motion.yaw_current_ecd <= chassis->rotate_motion.yaw_init_ecd)
	{
		chassis->rotate_motion.chassis_gimbal_angle = (float)(chassis->rotate_motion.yaw_init_ecd \
							    - chassis->rotate_motion.yaw_current_ecd)  \
								* GAMBAL_ENCODE_TO_ANGLE;
	}
	else if(chassis->rotate_motion.yaw_current_ecd > chassis->rotate_motion.yaw_init_ecd)
	{
		chassis->rotate_motion.   													   \
		chassis_gimbal_angle = 360.0f - (float)(chassis->rotate_motion.yaw_current_ecd \
										 - chassis->rotate_motion.yaw_init_ecd)        \
										 * GAMBAL_ENCODE_TO_ANGLE;
	}
#if 1
	chassis->forward_back_set = (int16_t)                                                         \
								((float)chassis->forward_back *  								  \
								 cos(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI) + \
							    (float)chassis->left_right * 									  \
								(-sin(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI)));
	
	chassis->left_right_set =   (int16_t)                                                         \
								((float)chassis->forward_back *  								  \
								 sin(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI) + \
							    (float)chassis->left_right * 									  \
								 cos(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI));
#else

#endif
	
}
/**
  * @brief        ��ȡ�ƶ�������
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void get_forward_back_value(chassis_control_data_t *chassis)
{
	int16_t speed = 0;
	if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_HIGH_SPEED_KEY)
	{
		speed = CHASSIS_MOUSE_CTRL_HIGH_SPPED;
	}
	else 
	{
		speed = CHASSIS_MOUSE_CTRL_NORMAL_SPPED;
	}


	if(chassis->connect->can2_rc_ctrl.control_mode == REMOTE_MODE)      
	{
		if ( RC_abs(chassis->connect->can2_rc_ctrl.rc.ch3) < 500 || RC_abs(chassis->connect->can2_rc_ctrl.rc.ch2) < 500)   //��������С��ʱ�������
		{
			if(chassis->connect->can2_rc_ctrl.rc.ch3 < 0 && chassis->connect->can2_rc_ctrl.rc.ch2 < 0)
			{	
				chassis->forward_back = -((chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500) *  \
										CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = -((chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500) *   \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
			else if (chassis->connect->can2_rc_ctrl.rc.ch3 < 0 )
			{
				chassis->forward_back = -((chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500) *  \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = (chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500 *   \
								CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
			else if ( chassis->connect->can2_rc_ctrl.rc.ch2 < 0 )
			{
				chassis->forward_back = (chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500 *  \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = -((chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500) *   \
								    CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
			else 
			{
				chassis->forward_back = (chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500 *  \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = (chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500 *   \
								    CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
		}
		else 
		{
			chassis->forward_back = chassis->connect->can2_rc_ctrl.rc.ch3 *    \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			chassis->left_right = chassis->connect->can2_rc_ctrl.rc.ch2 *      \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
		}
	}
	else if(chassis->connect->can2_rc_ctrl.control_mode ==  KEY_MOUSE_MODE)   //����ģʽ  *hyj
	{
		if(chassis->connect->can2_rc_ctrl.mouse.key & ROBOT_COMMON_MODE_KEY)       //��ͨ�����˶�
		{
			chassis->chassis_control_mode_flag = 0;

		}
		else if(chassis->connect->can2_rc_ctrl.mouse.key & ROBOT_RHOMB_MODE_KEY && 0)   //б���ε����˶� ���˴�ע�͵���б��
		{
			chassis->chassis_control_mode_flag = 0;
		}

		if(chassis->chassis_control_mode_flag)
		{
			speed = CHASSIS_MOUSE_CTRL_RHOMB_SPPED;
			//forward and back
			if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_FORWARD_KEY)//W
			{
				chassis->forward_back = speed*3/4;
				chassis->left_right = -speed*3/4;
			}
			else
			 if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_BACK_KEY)//S
			{
				chassis->forward_back = -speed*3/4;
				chassis->left_right = speed*3/4;
			}
			//left and right
			else if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_LEFT_KEY)//L
			{
				chassis->forward_back = -speed*3/4;
				chassis->left_right = -speed*3/4;

			}
			else if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_RIGHT_KEY)//R
			{
				chassis->forward_back = speed*3/4;
				chassis->left_right = speed*3/4;
			}
			else
			{
				chassis->forward_back = 0;
				chassis->left_right = 0;
			}	
		}
		else
		{
			//forward and back
			if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_FORWARD_KEY)//W
			{
				chassis->forward_back = speed*3/4;

			}
			else if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_BACK_KEY)//S
			{
				chassis->forward_back = -speed*3/4;

			}
			else
			{
				chassis->forward_back = 0;
			}
			//left and right
			if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_LEFT_KEY)//L
			{
				chassis->left_right = -speed*3/4;
			}
			else if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_RIGHT_KEY)//R
			{
				chassis->left_right = speed*3/4;
			}
			else
			{
				chassis->left_right = 0;
			}	
		}
	}	
}
/**
  * @brief         ��ȡ������תֵ
  * @author         
  * @param[in]      
  * @retval			
  * @note          
  */
float rotate_abs(float val)
{
	if(val < 0)
	{
		val = -val;
	}
	return val;
}
void get_rotate_value(chassis_control_data_t *chassis, chassis_pid_t *chassis_pid)
{
	if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_COMMON_MODE )//���̸���pid
	{
		if ( !correct_direction_flag )
		{
			chassis_pid->rotate_pid.set =(float)GAMBAL_YAW1_INIT_ENCODE_VALUE;//����ʼ����,����
		}
		else 
		{
			chassis_pid->rotate_pid.set =(float)GAMBAL_YAW2_INIT_ENCODE_VALUE;
		}
		
		chassis_pid->rotate_pid.fdb = (float)(gimbal_raw_value \
									+ ROBOT_COMMON_MODE_INS_YAW_offset_parameter*((chassis->connect->can2_rc_ctrl.gyro.yaw_set \
								+chassis->connect->can2_rc_ctrl.gyro.yaw_fdb)) );//+20.0f);// * GAMBAL_YAW_angle_VALUE)*0.08);


		chassis_pid->rotate_pid.Calc(&chassis_pid->rotate_pid);
		
		chassis->rotate = chassis_pid->rotate_pid.output;
		chassis->rotate_buff_flag = 0;

	} 
	else if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_ROTATE_MOTION_MODE)   //����С����    *hyj
	{
		if(freertos_run_time % 1000 == 0)      
        {
			srand(xTaskGetTickCount());
			chassis->rotate = rand() % CHASSIS_ROTATE_BUFF_SPEED + CHASSIS_ROTATE_BASE_SPEED ;
			chassis->rotate_buff_flag = 1;
		}  
		if(chassis->rotate_buff_flag != 1)     //�յ���Ĭ��Ϊ�����ٶ�       
		{
			chassis->rotate = CHASSIS_ROTATE_BASE_SPEED;
		}
	}
	else if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_ROTATE_STOP_MODE)
	{
		chassis->rotate = CHASSIS_ROTATE_STOP_SPEED;
		chassis->rotate_buff_flag = 0;
	}
	else 
	{
		chassis->rotate = 0;
	}
}
/**
  * @brief        ����+�������̷���
  * @author         
  * @param[in]      
  * @retval			
  * @note           ����ģʽ�������ģʽ����
  */
void caculate_gimbal_angle()
{
	if( ((chassis_control_data.connect->can2_rc_ctrl.mouse.key & ROBOT_CORRECT_DIRECTION_KEY) && chassis_control_data.connect->can2_rc_ctrl.control_mode == KEY_MOUSE_MODE) \
		|| (chassis_control_data.connect->can2_rc_ctrl.rc.ch3 == 660 && chassis_control_data.connect->can2_rc_ctrl.rc.ch2 == -660&& chassis_control_data.connect->can2_rc_ctrl.control_mode == GUI_CALI_MODE))
	{
		if( press_key_x_time < 1500 )
		{
			press_key_x_time ++ ;
		}
		if ( press_key_x_time < 1500 )
		{
			chassis_control_data.yaw_motor1_msg->encoder.round_cnt ++;
			chassis_control_data.yaw_motor2_msg->encoder.round_cnt ++;
			press_key_x_time = 1500;
		}
	}
	else
	{
		press_key_x_time = 0;
	}

	if ( !correct_direction_flag || 1)	// Yaw1�Ƕ�		һֱ����Yaw1
	{
		gimbal_round =  ((chassis_control_data.yaw_motor1_msg->encoder.round_cnt * 8192  \
						+ chassis_control_data.yaw_motor1_msg->encoder.raw_value)) / (8192+GAMBAL_YAW1_INIT_ENCODE_VALUE);	//12288���ݵ����ʼ������ֵ��ȷ��������״̬��raw_value�ĳ�ʼֵ��ȷ�� //12051=1*8192+row_value,�ø���raw_value��ֵ��ȷ��
		// gimbal_raw_value = ((chassis_control_data.yaw_motor1_msg->encoder.round_cnt * 8192 \
		// 				+ chassis_control_data.yaw_motor1_msg->encoder.raw_value)) % 16234 ; 
		gimbal_raw_value=chassis_control_data.yaw_motor1_msg->encoder.raw_value;
	}
	else							// Yaw2�Ƕ�(����û�õ�)
	{
		gimbal_round =  ((chassis_control_data.yaw_motor2_msg->encoder.round_cnt * 8192  \
						+ chassis_control_data.yaw_motor2_msg->encoder.raw_value)) / 11976 ;
		gimbal_raw_value = ((chassis_control_data.yaw_motor2_msg->encoder.round_cnt * 8192 \
						+ chassis_control_data.yaw_motor2_msg->encoder.raw_value)) % 11976 ;
	}
}
/**
  * @brief        ���µ��̵���趨ֵ�ͷ���ֵ
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void chassis_set_and_fdb_update(chassis_control_data_t *chassis, \
								chassis_pid_t *chassis_pid)
{
	caculate_gimbal_angle();
	switch(chassis->connect->can2_rc_ctrl.work_mode)	///ң�ؿ���ģʽѡ��
	{
		case ROBOT_CALI_MODE:
		case ROBOT_INIT_MODE:
		case ROBOT_INIT_END_MODE:
		{
			chassis->forward_back = 0; //ģʽת��ʱ����
			chassis->left_right = 0;
			chassis->rotate = 0;

			chassis->forward_back_set = 0;
			chassis->left_right_set = 0;
			chassis->rotate_set = 0;
		}break;
		case ROBOT_COMMON_MODE: //��ͨ���̸���ģʽ
		{
			get_forward_back_value(chassis);
			get_rotate_value(chassis, chassis_pid);
			
			chassis->forward_back_set = chassis->forward_back;
			chassis->left_right_set = chassis->left_right;
			chassis->rotate_set = chassis->rotate;
		}break;
		case ROBOT_ROTATE_MOTION_MODE: //�˶�С����ģʽ
		{
			get_forward_back_value(chassis);//��ȡ����ֵ����ʹ�����溯����ת��
			rotate_motion_mode_process(chassis);//�˶�С���ݽ���
			get_rotate_value(chassis, chassis_pid);
			
			chassis->rotate_set =  - chassis->rotate;
		}break;
		case ROBOT_ROTATE_STOP_MODE: //��ֹ����С����ģʽ
		{
			get_rotate_value(chassis, chassis_pid);
			
			chassis->forward_back_set = 0;
			chassis->left_right_set = 0;
			chassis->rotate_set =  - chassis->rotate;
		}break;
		default:
		{
			chassis->forward_back = 0;
			chassis->left_right = 0;
			chassis->rotate = 0;
			
			chassis->forward_back_set = 0;
			chassis->left_right_set = 0;
			chassis->rotate_set = 0;
		}break;
	}
	
#if 0
	chassis->rotate_set = 0; //��������ʹ�� ����Ҫ��ת��
#endif	
	
	//cm1 Ϊ���ϵ�� ������ʱ��
	chassis->cm1_set = - chassis->forward_back_set + chassis->left_right_set + chassis->rotate_set;
	chassis->cm2_set = chassis->forward_back_set + chassis->left_right_set + chassis->rotate_set;
	chassis->cm3_set = chassis->forward_back_set - chassis->left_right_set + chassis->rotate_set;
	chassis->cm4_set = - chassis->forward_back_set - chassis->left_right_set + chassis->rotate_set;
	
	chassis->cm1_fdb = chassis->cm1_msg->encoder.filter_rate;
	chassis->cm2_fdb = chassis->cm2_msg->encoder.filter_rate;
	chassis->cm3_fdb = chassis->cm3_msg->encoder.filter_rate;
	chassis->cm4_fdb = chassis->cm4_msg->encoder.filter_rate;
	
	if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_INIT_MODE)
	{
		HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
	}
}

/**********************����ע���ǲ��ӷ����Ĵ���**************************/
//void magazine_control(chassis_control_data_t *chassis)
//{
//	if (chassis->magazine_control_flag)
//	{
//		open_magazine();
//	}
//	else
//	{
//		close_magazine();
//	}

//	if (chassis->thumbwheel_move_flag )
//	{
//		chassis->thumbwheel_speed_set = thumbwheel_SPEED_RATIO / thumbwheel_SPEED_GRID * thumbwheel_freq;	//244.8 ///װ���ٶ�
//	}	
//	else
//	{
//		chassis->thumbwheel_speed_set = 0;
//	}
//	chassis->thumbwheel_speed_fdb = chassis->thumbwheel_motor_msg->encoder.filter_rate;

//}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note        �Զ����������̷���������������һ��ʱ�䣬֮������жϵ���ת���������0��
  */
  uint8_t last_thumbwheel_remote_flag;
void magazine_control(chassis_control_data_t *chassis)
{
	if (chassis->magazine_control_flag)
	{
		open_magazine();
	}
	else
	{
		close_magazine();
	}
	
	if(abs(chassis->thumbwheel_motor_msg->encoder.filter_rate)<16)	//���̽��ٶȻ�����������Ϊ�Ƕ�ת
	{
			chassis->thumbwheel_stop_flag=1;
	}
	else
	{
			chassis->thumbwheel_stop_flag=0;
	}

	if (chassis->thumbwheel_move_flag )
	{
		  if (chassis->thumbwheel_contrary_flag==1)
			 {			 
			    if (chassis->thumbwheel_remote_flag==1&&last_thumbwheel_remote_flag==0)	//ң������һ�����ʼ�����в�����ر�־λ
			    {
					//chassis->thumbwheel_stop_flag=0;
					chassis->thumbwheel_forwad_flag=0;
			     }
				last_thumbwheel_remote_flag = chassis->thumbwheel_remote_flag;				
			  }
			
		  if (chassis->thumbwheel_forwad_flag<=1)//��ת����֮�󣬲�����ת��־�ۼ���250�β�ֹͣ��ת
		   {
			   chassis->thumbwheel_forwad_flag++;
			    chassis->thumbwheel_speed_set = (thumbwheel_SPEED_RATIO / thumbwheel_SPEED_GRID * thumbwheel_freq);	//244.8 ///װ���ٶ�	
		   }
		   else
		   {
			   chassis->thumbwheel_acc_flag=0;
			   chassis->thumbwheel_speed_set=0;
		   }
//		  else if(chassis->thumbwheel_acc_flag!=0)	//��ת
//		   {
//			    chassis->thumbwheel_acc_flag++;			
//			    chassis->thumbwheel_speed_set = -(thumbwheel_SPEED_RATIO / thumbwheel_SPEED_GRID * thumbwheel_freq);			 
//			    if (chassis->thumbwheel_acc_flag>=100)	//��תһ��ʱ���˳���ת
//			    {
//				    chassis->thumbwheel_acc_flag=0;
//				}
//			}
//			if(chassis->thumbwheel_remote_flag==0&&chassis->thumbwheel_stop_flag==1)//������ת��־�ۼ�250��֮������жϵ���ת���ò��̵�������0	
//			{
//				chassis->thumbwheel_speed_set=0;
//			}
//			if(chassis->thumbwheel_remote_flag==1&&chassis->thumbwheel_stop_flag==0)
//			{
//				chassis->thumbwheel_speed_set=0;
//			}
       chassis->thumbwheel_speed_fdb = chassis->thumbwheel_motor_msg->encoder.filter_rate;

		
   }
 }
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void chassis_pid_calculate(chassis_control_data_t *chassis,  \
						   chassis_pid_t *chassis_pid)
{
	chassis_pid->cm1_pid.set = chassis->cm1_set;
	chassis_pid->cm2_pid.set = chassis->cm2_set;
	chassis_pid->cm3_pid.set = chassis->cm3_set;
	chassis_pid->cm4_pid.set = chassis->cm4_set;
	chassis_pid->thumbwheel_pid.set = chassis->thumbwheel_speed_set;

	
	chassis_pid->cm1_pid.fdb = chassis->cm1_fdb;
	chassis_pid->cm2_pid.fdb = chassis->cm2_fdb;
	chassis_pid->cm3_pid.fdb = chassis->cm3_fdb;
	chassis_pid->cm4_pid.fdb = chassis->cm4_fdb;
	chassis_pid->thumbwheel_pid.fdb = chassis->thumbwheel_speed_fdb;


	
	chassis_pid->cm1_pid.Calc(&chassis_pid->cm1_pid);
	chassis_pid->cm2_pid.Calc(&chassis_pid->cm2_pid);
	chassis_pid->cm3_pid.Calc(&chassis_pid->cm3_pid);
	chassis_pid->cm4_pid.Calc(&chassis_pid->cm4_pid);
	chassis_pid->thumbwheel_pid.Calc(&chassis_pid->thumbwheel_pid);
}

void chassis_forwardfeed(chassis_control_data_t *chassis)
{
 chassis->cm1_ff=forwardfeed(RC_abs(chassis->cm1_msg->encoder.filter_rate) , &chassis_control_data);
 chassis->cm2_ff=forwardfeed(RC_abs(chassis->cm2_msg->encoder.filter_rate) , &chassis_control_data);
	chassis->cm3_ff=forwardfeed(RC_abs(chassis->cm3_msg->encoder.filter_rate) , &chassis_control_data);
	chassis->cm4_ff=forwardfeed(RC_abs(chassis->cm4_msg->encoder.filter_rate) , &chassis_control_data);
	if(chassis->cm1_msg->encoder.filter_rate>0)
	{
	chassis->cm2_ff=-chassis->cm2_ff;
	chassis->cm3_ff=-chassis->cm3_ff;		
	}
	else
	{
	chassis->cm1_ff=-chassis->cm1_ff;
	chassis->cm4_ff=-chassis->cm4_ff;
	}
		
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
 
void chassis_control_loop(chassis_control_data_t *chassis, \
						  chassis_pid_t *chassis_pid)
{
	chassis->given_current.cm1 = chassis_pid->cm1_pid.output + chassis->cm1_ff;
	chassis->given_current.cm2 = chassis_pid->cm2_pid.output + chassis->cm2_ff;
	chassis->given_current.cm3 = chassis_pid->cm3_pid.output + chassis->cm3_ff;
	chassis->given_current.cm4 = chassis_pid->cm4_pid.output + chassis->cm4_ff;
	chassis->given_current.thumbwheel =( chassis_pid->thumbwheel_pid.output);	
	
	if(chassis->connect->can2_rc_ctrl.control_mode == GUI_CALI_MODE )	///����У׼ģʽ������ֹͣ�˶�
	{
		set_chassis_stop();
		set_thumbwheel_behaviour(0);									///����	
	}
	else 
	{
		set_chassis_behaviour(chassis->given_current.cm1,
							  chassis->given_current.cm2,
							  chassis->given_current.cm3,
							  chassis->given_current.cm4);
//		set_thumbwheel_behaviour(chassis->given_current.thumbwheel)

	}
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
extern osThreadId_t GUI_task_Handler;
void set_GUI_task_state(chassis_control_data_t *chassis)
{
	if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_CALI_MODE)
	{
		if(eTaskGetState(GUI_task_Handler) == eSuspended)//����״̬
		{
			LED_Init(); // OLED��Ļ��ʼ������ֹ��Ļ����
			osThreadResume(GUI_task_Handler);//����ָ�
			LED_Fill(0x00);
			LED_Init();
			LED_Fill(0x00);
		}
	}
	else
	{
		if(eTaskGetState(GUI_task_Handler) != eSuspended)
		{	
			osThreadSuspend(GUI_task_Handler);
			LED_Fill(0x00);
			LED_Init();
			LED_Fill(0x00);
		}
	}
} 

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note
  */

void chassis_task(void *argument)
{
	TickType_t current_time = 0;

	vTaskDelay(CHASSIS_TASK_INIT_TIME);								///�����ʱ���������Զ���ʱ�����ں�������״̬���ó�CPU�������ȼ�������ִ�У�ִ����������ڲ�ȷ��
	chassis_init(&chassis_control_data, &chassis_pid);
	chassis_control_data.yaw_motor1_msg->encoder.round_cnt = 0;
	chassis_control_data.yaw_motor2_msg->encoder.round_cnt = 24;	//yaw�����������
	while(1)
	{
		current_time = xTaskGetTickCount();                         //��ǰϵͳʱ��       *hyj
		chassis_power_limit();								
		magazine_control(&chassis_control_data);			// ���ֿ���
		chassis_set_and_fdb_update(&chassis_control_data, &chassis_pid);
		chassis_pid_calculate(&chassis_control_data, &chassis_pid);	///����PID���㣬ǰ�߸�ֵ������
		chassis_control_loop(&chassis_control_data, &chassis_pid);	///����״̬�������ƣ�У׼ģʽ�ͷ�У׼ģʽ�£������߸�ֵ��ǰ��
		if(chassis_control_data.connect->can2_rc_ctrl.work_mode == ROBOT_CALI_MODE)		// ����ûд�ã��ǵ��԰�GUI����	
		{
			set_GUI_task_state(&chassis_control_data);	///����У׼״̬�ſ��Խ���GUI������
		}
		vTaskDelayUntil(&current_time, 1);       //1msһ��         *hyj///������ʱ������ִ����������ھ��ԣ����ȼ��ߵĻ�ʱ�䵽�����������
	}
}  

