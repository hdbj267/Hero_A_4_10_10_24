#include "GUI_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "oled.h"
#include "remote_app.h"
#include "pid.h"
#include "judge.h"
#include "adc.h"
#include "rule.h"

rc_analog_key_t rc_analog_key;
extern chassis_control_data_t chassis_control_data;
extern chassis_pid_t chassis_pid;
extern motor_msg_t yaw_motor1_msg;
extern motor_msg_t yaw_motor2_msg;
extern connect_t connect_data;
extern ext_game_robot_status_t robot_status;
extern judge_manegement_t judge_manegement;
extern ext_shoot_data_t shoot_data;
extern ext_game_robot_pos_t robot_position;
extern ext_power_heat_data_t power_heat;
extern ext_bullet_remaining_t bullets_remaining;
extern TaskHandle_t chassis_task_Handler;
extern robot_t Infantry;
/*
A板上的灯通过定时器7闪烁
*/
uint16_t Led_blink_time=0;
uint8_t Led_blink_flag;
uint8_t Led_blink_state = 1;
void LED_blink()
{
	Led_blink_time++;
	if(Led_blink_time>500)
	{
		Led_blink_time=0;
		Led_blink_state = Led_blink_state ^ 1;
		HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,Led_blink_state);
	}
}

void get_rc_analoy_key_value(rc_analog_key_t *analoy_key, RC_ctrl_t *rc_ctrl_data)
{
	if(analoy_key == NULL || rc_ctrl_data == NULL)
	{
		return;
	}
	if(rc_ctrl_data->rc.ch3>ANALOY_KEY_THRE_VALUE)  analoy_key->key_up = 1;//上
	else analoy_key->key_up = 0;
	
	if(rc_ctrl_data->rc.ch3<(-ANALOY_KEY_THRE_VALUE))  analoy_key->key_down = 1;//下
	else analoy_key->key_down = 0;
	
	if(rc_ctrl_data->rc.ch2>ANALOY_KEY_THRE_VALUE)  analoy_key->key_back = 1;//右
	else analoy_key->key_back = 0;
	
	if(rc_ctrl_data->rc.ch2<(-ANALOY_KEY_THRE_VALUE))  analoy_key->key_ok = 1;//左
	else analoy_key->key_ok = 0;
}
void rc_analoy_key_init(void)
{
	rc_analog_key.key_back = 0;
	rc_analog_key.key_down = 0;
	rc_analog_key.key_ok = 0;
	rc_analog_key.key_up = 0;
	rc_analog_key.rc_ctrl_data_point = get_rc_data_point();
}
/****************************OLED调试所用变量******************************/
uint8_t key_num = 0;
#define     oled_key_release    0
#define     oled_key_up         1
#define     oled_key_down       2
#define     oled_key_left       3
#define     oled_key_right      4
#define     oled_key_center     5
extern int16_t gimbal_round ;
extern uint16_t gimbal_raw_value ;
void Motor_message()
{
	LED_Fill(0x00);
	while(1)
	{
		LED_P6x8Str(28,0,(uint8_t *)"Motor_message");
		LED_P6x8Str(3,1,(uint8_t *)"CM_1:");
		LED_PrintValueI(40,1,cm1_msg.encoder.filter_rate);
		LED_P6x8Str(3,2,(uint8_t *)"CM_2:");
		LED_PrintValueI(40,2,cm2_msg.encoder.filter_rate);
		LED_P6x8Str(3,3,(uint8_t *)"CM_3:");
		LED_PrintValueI(40,3,cm3_msg.encoder.filter_rate);
		LED_P6x8Str(3,4,(uint8_t *)"CM_4:");
		LED_PrintValueI(40,4,cm4_msg.encoder.filter_rate);

		LED_P6x8Str(3,5,(uint8_t *)"Yaw1:");
		LED_PrintValueI(40,5,yaw_motor1_msg.encoder.raw_value);
		LED_P6x8Str(3,6,(uint8_t *)"Yaw2:");
		LED_PrintValueI(40,6,yaw_motor2_msg.encoder.raw_value);

		LED_P6x8Str(3,7,(uint8_t *)"thumb");
		LED_PrintValueI(40,7,chassis_control_data.thumbwheel_motor_msg->encoder.filter_rate);
		
		if(key_scanf()==oled_key_left)
		{
			while(key_scanf());	//等待按键松开
			break;
		}
		osDelay(100);
	}
}
void Robot_message(void)
{
	LED_Fill(0x00);
	while(1)
	{
		LED_P6x8Str(28,0,(uint8_t *)"Robot_message");
		LED_P6x8Str(3,1,(uint8_t *)"ID:");
		LED_PrintValueC(54,1,robot_status.robot_id );
		LED_P6x8Str(3,2,(uint8_t *)"HP:");
		LED_PrintValueI(54,2,robot_status.remain_HP );
		LED_P6x8Str(3,3,(uint8_t *)"Level:");
		LED_PrintValueC(54,3,robot_status.robot_level );
		LED_P6x8Str(3,4,(uint8_t *)"Pre_HP:");
		LED_PrintValueI(54,4, Infantry.Pre_HP );
		LED_P6x8Str(3,5,(uint8_t *)"Hurt_tp:");
		LED_PrintValueI(54,5, robot_hurt.hurt_type );
		if(key_scanf()==oled_key_left)
		{
			while(key_scanf());	//等待按键松开
			break;
		}
		osDelay(100);
	}
}
void shoot_message(void)
{
	LED_Fill(0x00);
	while(1)
	{
		LED_P6x8Str(28,0,(uint8_t *)"shoot_message");
		LED_P6x8Str(3,1,(uint8_t *)"type:");
		LED_PrintValueI(40,1,shoot_data.bullet_type);
		LED_P6x8Str(3,2,(uint8_t *)"freq:");
		LED_PrintValueI(50,2,shoot_data.bullet_freq);
		LED_P6x8Str(3,3,(uint8_t *)"speed:");
		LED_PrintValueF(50,3,shoot_data.bullet_speed,2);
		LED_P6x8Str(3,4,(uint8_t *)"heat17:");	// 17mm 枪口热量
		LED_PrintValueI(50,4,power_heat.shooter_id1_17mm_cooling_heat);
		LED_P6x8Str(3,5,(uint8_t *)"V_lim:");
		LED_PrintValueI(50,5,robot_status.shooter_id1_17mm_speed_limit);
		LED_P6x8Str(3,6,(uint8_t *)"H_lim:");
		LED_PrintValueI(50,6,robot_status.shooter_id1_17mm_cooling_limit);
		LED_P6x8Str(3,7,(uint8_t *)"count:");	
		LED_PrintValueI(50,7,bullets_remaining.shootnum);	//统计发弹量,每触发一次则认为发射了一颗
		if(key_scanf()==oled_key_left)
		{
			while(key_scanf());	//等待按键松开
			break;
		}
		osDelay(100);
	}
}
void power_message(void)
{
	LED_Fill(0x00);
	while(1)
	{
		LED_P6x8Str(28,0,(uint8_t *)"power_message:");
		LED_P6x8Str(3,1,(uint8_t *)"volt:");	// 底盘输出电压 单位 毫伏
		LED_PrintValueI(40,1,power_heat.chassis_volt);
		LED_P6x8Str(3,2,(uint8_t *)"curre:");	// 底盘输出电流 单位 毫安
		LED_PrintValueI(50,2,power_heat.chassis_current);
		LED_P6x8Str(3,3,(uint8_t *)"power:");	// 底盘输出功率 单位 W 瓦 
		LED_PrintValueF(50,3,power_heat.chassis_power,2);
		LED_P6x8Str(3,4,(uint8_t *)"buff:");	// 底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
		LED_PrintValueI(40,4,power_heat.chassis_power_buffer);
		LED_P6x8Str(3,5,(uint8_t *)"limit:");	// 底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
		LED_PrintValueI(40,5,robot_status.chassis_power_limit);
		/*
		// 底盘输出电压 单位 毫伏
		// 底盘输出电流 单位 毫安
		// 底盘输出功率 单位 W 瓦 
		// 底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
		// 17mm 枪口热量
		// 42mm 枪口热量
		*/
		if(key_scanf()==oled_key_left)
		{
			while(key_scanf());	//等待按键松开
			break;
		}
		osDelay(100);
	}
}
void GYRO_ch23_message()
{
	LED_Fill(0x00);
	while(1)
	{
		LED_P6x8Str(10,1,(uint8_t *)"GYRO_set:");
		LED_PrintValueF(65,1,connect_data.can2_rc_ctrl.gyro.yaw_set,2);

		LED_P6x8Str(10,2,(uint8_t *)"GYRO_fdb");
		LED_PrintValueF(65,2,connect_data.can2_rc_ctrl.gyro.yaw_fdb,2);

		LED_P6x8Str(10,3,(uint8_t *)"ch2");
		LED_PrintValueI(65,3,connect_data.can2_rc_ctrl.rc.ch2);

		LED_P6x8Str(10,4,(uint8_t *)"ch3");
		LED_PrintValueI(65,4,connect_data.can2_rc_ctrl.rc.ch3);

		LED_P6x8Str(10,5,(uint8_t *)"round");
		LED_PrintValueI(65,5,gimbal_round);

		LED_P6x8Str(10,6,(uint8_t *)"raw_value");
		LED_PrintValueI(65,6,gimbal_raw_value);

		if(key_scanf()==oled_key_left)
		{
			while(key_scanf());	//等待按键松开
			break;
		}
		osDelay(100);
	}
}

void Motor_test(void)
{
	int16_t test_motor1_duty=0, test_motor2_duty=0, test_motor3_duty=0, test_motor4_duty=0;
	int choose_num=1;
	LED_Fill(0x00);
	osThreadSuspend(chassis_task_Handler);//任务挂起
	while(1)
	{
		LED_P6x8Char(3, choose_num, '*');
		LED_P6x8Str(28,0,(uint8_t *)"power_message:");
		LED_P6x8Str(10,1,(uint8_t *)"Motor1");
		LED_P6x8Str(10,2,(uint8_t *)"Motor2:");
		LED_P6x8Str(10,3,(uint8_t *)"Motor3:");
		LED_P6x8Str(10,4,(uint8_t *)"Motor4:");
		LED_P6x8Str(10,5,(uint8_t *)"Null:");
		LED_P6x8Str(10,6,(uint8_t *)"Null:");
		LED_P6x8Str(10,7,(uint8_t *)"Return:");
		while(!key_scanf());
		delay_ms(20);
		key_num = key_scanf();
		while(key_scanf());
		delay_ms(100);
		if(key_num == oled_key_up)
		{
			choose_num++;
			if(choose_num==8)
				choose_num=1;
		}
		else if (key_num == oled_key_up)
		{
			choose_num--;
			if(choose_num==0)
				choose_num=7;
		}
		else if(key_num == oled_key_left)
		{
			switch (choose_num)
			{
			case 1:
				test_motor1_duty-=100;break;
			case 2:
				test_motor2_duty-=100;break;
			case 3:
				test_motor3_duty-=100;break;
			case 4:
				test_motor4_duty-=100;break;
			default:
				break;
			}
		}
		else if(key_num == oled_key_right)
		{
			switch (choose_num)
			{
			case 1:
				test_motor1_duty+=100;break;
			case 2:
				test_motor2_duty+=100;break;
			case 3:
				test_motor3_duty+=100;break;
			case 4:
				test_motor4_duty+=100;break;
			case 5:
				break;
			case 6:
				break;
			default:
				break;
			}
		}
		else if(key_scanf()==oled_key_center && choose_num==7)
		{
			while(key_scanf());	//等待按键松开
			osThreadResume(chassis_task_Handler);//任务恢复
			LED_Fill(0x00);
			break;
		}
		set_chassis_behaviour(test_motor1_duty,\
							  test_motor2_duty,\
							  test_motor3_duty,\
							  test_motor4_duty);
		osDelay(10);
	}
}
char task_ch[400] = {0};

/**
  * @brief         主菜单选择指 示，控制星号即指示标的 跳动  人机交互总函数   
  * @author         
  * @param[in]      
  * @retval		   返回空
  * @note
  */
void GUI_interaction(void)                 
{
    int choose_num = 1;
	while(1)
	{
		LED_Fill(0x00);
		LED_P6x8Str(52,0,(uint8_t *)"MENU");
    	LED_P6x8Str(10,1,(uint8_t *)"Robot_message");
		LED_P6x8Str(10,2,(uint8_t *)"Motor_message");
    	LED_P6x8Str(10,3,(uint8_t *)"shoot_message");
		LED_P6x8Str(10,4,(uint8_t *)"power_message");
		LED_P6x8Str(10,5,(uint8_t *)"Motor_test_Null");
		LED_P6x8Str(10,6,(uint8_t *)"GYRO_ch23_message");
	  	LED_P6x8Char(3, choose_num, '*');
		// memset(task_ch,0,400);
		// vTaskGetRunTimeStats(task_ch);

		while(!key_scanf());
		delay_ms(20);
		key_num = key_scanf();
		while(key_scanf());
		delay_ms(20);
		if(key_num == oled_key_up)
		{
			choose_num--;
			if(choose_num==0)
				choose_num=6;
		}
		else if(key_num == oled_key_down)
		{
			choose_num++;
			if(choose_num==7)
				choose_num=1;
		}
		else if(key_num == oled_key_center)
		{
			switch(choose_num)
			{
				case 1:Robot_message();break;
				case 2:Motor_message();break;
				case 3:shoot_message();break;
				case 4:power_message();break;
				case 5:Motor_test();break;
				case 6:GYRO_ch23_message();break;
				// case 6:Robot_message();break;
				// case 7:Robot_message();break;
				default:break;
			}
		}
		vTaskDelay(100);
	}
}

void GUI_task(void *argument)
{
	vTaskDelay(GUI_TASK_INIT_TIME);
	LED_Init(); // OLED屏幕初始化
	rc_analoy_key_init();	//接收的遥控器控制数据，选择进入人机交互界面
	while(1)
	{
		GUI_interaction();	//人机交互界面
	}
}



















cali_gimbal_t cali_gimbal_pid = 
{
/**
* @brief yaw pid param     PID_POSITION = 0,PID_DELTA = 1
* @note    
*/
	
	.yaw_pid.position.kp = 25.0,
	.yaw_pid.position.ki = 0,
	.yaw_pid.position.kd = 0,
	.yaw_pid.position.ioutput_max = 1000,
	.yaw_pid.position.output_max = 5000,
	.yaw_pid.position.mode = PID_POSITION,	
	
	.yaw_pid.speed.kp = 25.0,
	.yaw_pid.speed.ki = 0.5,//25.0   0.5
	.yaw_pid.speed.kd = 0,
	.yaw_pid.speed.ioutput_max = 1000,
	.yaw_pid.speed.output_max = 5000,
	.yaw_pid.speed.mode = PID_POSITION,	
/**
* @brief pitch pid param         
* @note    
*/
	.pitch_pid.position.kp = 25.0,
	.pitch_pid.position.ki = 0,
	.pitch_pid.position.kd = 0,
	.pitch_pid.position.ioutput_max = 1000,
	.pitch_pid.position.output_max = 5000,
	.pitch_pid.position.mode = PID_POSITION,	
	
	.pitch_pid.speed.kp = 25.0,
	.pitch_pid.speed.ki = 0.5,
	.pitch_pid.speed.kd = 0,
	.pitch_pid.speed.ioutput_max = 1000,
	.pitch_pid.speed.output_max = 5000,
	.pitch_pid.speed.mode = PID_POSITION,	

};

cali_shoot_t cali_shoot_pid = 
{
	/**
* @brief trigger pid param         
* @note    
*/	
	.trigger_pid.position.kp = 15.0,
	.trigger_pid.position.ki = 0,
	.trigger_pid.position.kd = 0,
	.trigger_pid.position.ioutput_max = 1000,
	.trigger_pid.position.output_max = 5000,
	.trigger_pid.position.mode = PID_POSITION,	
	
	.trigger_pid.speed.kp = 15.0,
	.trigger_pid.speed.ki = 0.5,
	.trigger_pid.speed.kd = 0,
	.trigger_pid.speed.ioutput_max = 1000,
	.trigger_pid.speed.output_max = 5000,
	.trigger_pid.speed.mode = PID_POSITION,	
/**
* @brief fric1 pid param         
* @note    
*/	
	.fric1_pid.kp = 10,
	.fric1_pid.ki = 0,
	.fric1_pid.kd = 0,
	.fric1_pid.ioutput_max = 1000,
	.fric1_pid.output_max = 5000,
	.fric1_pid.mode = PID_POSITION,	//PID_DELTA
/**
* @brief fric2 pid param        
* @note    
*/	
	.fric2_pid.kp = 10,
	.fric2_pid.ki = 0,
	.fric2_pid.kd = 0,
	.fric2_pid.ioutput_max = 1000,
	.fric2_pid.output_max = 5000,
	.fric2_pid.mode = PID_POSITION,	
};
/*****以下注释的代码被放到chassis_task.c******/
//cali_chassis_t cali_chassis_pid = 
//{
///**
//* @brief fric1 pid param         
//* @note    
//*/	
//	.cm_pid.kp = 20.0,  //6.2  //10stable  20
//	.cm_pid.ki = 0.0,  //0.3   //0
//	.cm_pid.kd = 0.0,
//	.cm_pid.ioutput_max = 1000,
//	.cm_pid.output_max = 5000,
//	.cm_pid.mode = PID_POSITION,			//PID_DELTA	PID_POSITION
//	
//	.rotate_pid.kp = 1.0,
//	.rotate_pid.ki = 0.0,
//	.rotate_pid.kd = 30,
//	.rotate_pid.ioutput_max = 1000,
//	.rotate_pid.output_max = 5000,
//	.rotate_pid.mode = PID_POSITION,	//PID_POSITION	//待调试确定 目前抖动很大
//};
/*****以下注释的代码被放到chassis_task.c******/

