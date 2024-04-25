#ifndef GUI_TASK_H
#define GUI_TASK_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "remote_app.h"
#include "chassis_task.h"

#define GUI_TASK_INIT_TIME       (10)
#define ANALOY_KEY_THRE_VALUE    (50) 

typedef struct
{
	int16_t key_up;
	int16_t key_down;
	int16_t key_ok;
	int16_t key_back;
	void *rc_ctrl_data_point;
}rc_analog_key_t;
/*****以下注释的代码被放到chassis_task.h******/
//typedef struct
//{
//	float kp;
//	float ki;
//	float kd;
//	
//	float ioutput_max;
//	float output_max;
//	uint8_t mode;
//}cali_pid_t;

//typedef struct
//{
//	cali_pid_t position;
//	cali_pid_t speed;
//}cali_cascade_pid_t;
/*****以上注释的代码被放到chassis_task.h******/
typedef struct
{
	cali_cascade_pid_t yaw_pid;
	cali_cascade_pid_t pitch_pid;
}cali_gimbal_t;

typedef struct
{	
	cali_cascade_pid_t trigger_pid;
	cali_pid_t fric1_pid;
	cali_pid_t fric2_pid;
}cali_shoot_t;
/*****以下注释的代码被放到chassis_task.h******/
//typedef struct
//{
//	cali_pid_t cm_pid;
//	cali_pid_t rotate_pid;
//}cali_chassis_t;
/*****以上注释的代码被放到chassis_task.h******/
extern rc_analog_key_t rc_analog_key;

extern cali_gimbal_t cali_gimbal_pid;
extern cali_shoot_t cali_shoot_pid;
extern cali_chassis_t cali_chassis_pid;

void rc_analoy_key_init(void);
void get_rc_analoy_key_value(rc_analog_key_t *rc_analoy_key, RC_ctrl_t *rc_ctrl_data);
void GUI_task(void *argument);
void LED_blink(void);

#endif
