#ifndef GIMBAL_APP_H
#define GIMBAL_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "pid.h"
#include "connect_task.h"
 
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)

#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

#define CHASSIS_FORWARD_KEY              KEY_PRESSED_OFFSET_W           
#define CHASSIS_BACK_KEY                 KEY_PRESSED_OFFSET_S        
#define CHASSIS_LEFT_KEY                 KEY_PRESSED_OFFSET_A           
#define CHASSIS_RIGHT_KEY                KEY_PRESSED_OFFSET_D    
#define CHASSIS_HIGH_SPEED_KEY           KEY_PRESSED_OFFSET_SHIFT
#define ROBOT_COMMON_MODE_KEY            KEY_PRESSED_OFFSET_CTRL  
#define ROBOT_RHOMB_MODE_KEY             KEY_PRESSED_OFFSET_C
#define ROBOT_CORRECT_DIRECTION_KEY      KEY_PRESSED_OFFSET_X


#define CHASSIS_TASK_INIT_TIME           (200)                        //������ǰ��ʱ ��λms
#define CHASSIS_RC_CTRL_SPPED_MAX_FACT   (1.0f)//2.2f //1.2f����ܶ���������//ҡ���ٶȲ��� 0.45 //�����ٶ����� speed_max = 660*STICK_TO_CHASSIS_SPEED_REF_FACT

#define CHASSIS_MOUSE_CTRL_NORMAL_SPPED  (500.0f)                    //������ʱ�� �ٶ�
#define CHASSIS_MOUSE_CTRL_HIGH_SPPED     (1000.0f)                  //��shift�ļ���ֵ
#define CHASSIS_MOUSE_CTRL_RHOMB_SPPED   (400.0f)                    //б�����ٶ�
#define GAMBAL_YAW_INIT_ENCODE_VALUE_COMMON     (5450)               //��ͨ�����˶�������ı���ֵ
#define GAMBAL_YAW_INIT_ENCODE_VALUE_RHOMB      (6470)               //б�����˶�������ı���ֵ
 
#define GAMBAL_YAW1_INIT_ENCODE_VALUE     (3859)     //5950 //����YAWʱ������Ҫ����ʵ�ʱ���ֵ����   
#define GAMBAL_YAW2_INIT_ENCODE_VALUE      (3847)	//5254  

#define GAMBAL_YAW_angle_VALUE     (22.755555f)                      //yaw	1 �� 1.5		1.72     34.133333f=(8192/360)*1.5-->1:1.5//����ʱ1��1�ļ�����

#define thumbwheel_SPEED_RATIO     153       //�����һ��תһȦ,152ת��ת��482*19/60=152,����Ƶ�ٳ��Բ��̸����Ϳɵ���Ӧ��Ƶ�µ�ת��
#define thumbwheel_SPEED_GRID      6			//���̸���
#define thumbwheel_freq				20	// ����Ƶ��  ��һ�� 10


#define CAMBAL_PITCH_MAX_ANGLE_TO_ENCODE (3200.0f)
#define CAMBAL_PITCH_MIN_ANGLE_TO_ENCODE (4285.0f)
#define GAMBAL_ENCODE_TO_ANGLE           ((float)0.0439506775729459)  //  360.0/8191.0
	
#define CHASSIS_ROTATE_MOTION_SPEED      (600u)//(300u)               //�˶�С���ݵ�rotateֵ
#define CHASSIS_ROTATE_BASE_SPEED        (600u)                        //�˶�С���ݵ�rotateֵ
#define CHASSIS_ROTATE_BUFF_SPEED        (600u)                       //�˶�С���ݵ�rotateֵ
#define CHASSIS_ROTATE_STOP_SPEED        (1000u)                       //��ֹС���ݵ�rotateֵ

#define PI                               (3.14159265358979f)

#define ROBOT_COMMON_MODE_INS_YAW_offset_parameter 20  //���̸���ʱ����������yaw�����Ĳ��� 

typedef enum
{
	ROBOT_CALI_MODE = 0,     //����ģʽ
	ROBOT_INIT_MODE,         //��ʼ��
	ROBOT_INIT_END_MODE,     //��ʼ�������л���
	ROBOT_COMMON_MODE,       //��ͨ���̸���ģʽ
	ROBOT_ROTATE_STOP_MODE,  //��ֹС����
	ROBOT_ROTATE_MOTION_MODE,//�˶�С����
	ROBOT_ERROR_MODE,        //����
}robot_work_mode_e;

typedef enum
{
	KEY_MOUSE_MODE = 0,
	REMOTE_MODE,
	GUI_CALI_MODE,
}robot_control_mode_e;

typedef struct
{
	int16_t cm1;
	int16_t cm2;
	int16_t cm3;
	int16_t cm4;
	int16_t thumbwheel;
}given_current_t;

typedef struct
{
	pid_t cm1_pid;	
	pid_t cm2_pid;
	pid_t cm3_pid;	
	pid_t cm4_pid;	
	
	pid_t rotate_pid;

	pid_t thumbwheel_pid;
}chassis_pid_t;

typedef struct
{
	int16_t yaw_current_ecd;
	int16_t yaw_init_ecd;
	float chassis_gimbal_angle;
}rotate_motion_t;

typedef struct
{
	const connect_t *connect;
	const motor_msg_t *cm1_msg;
	const motor_msg_t *cm2_msg;
	const motor_msg_t *cm3_msg;
	const motor_msg_t *cm4_msg;
	const motor_msg_t *thumbwheel_motor_msg;
	motor_msg_t *yaw_motor1_msg;
	motor_msg_t *yaw_motor2_msg;
	
	given_current_t given_current;//��������͵ĵ���ֵ
	rotate_motion_t rotate_motion;//С�����˶�ר�ýṹ��
	
	int16_t forward_back;
    int16_t left_right;
    int16_t rotate;
	int16_t rotate_buff_flag;           //��̬���ٱ�־
	
	int16_t forward_back_set;
    int16_t left_right_set;
    int16_t rotate_set;
	
	float cm1_set;
	float cm2_set;
	float cm3_set;
	float cm4_set;
	
	float cm1_ff;
	float cm2_ff;
	float cm3_ff;
	float cm4_ff;
	
	float cm1_fdb;
	float cm2_fdb;
	float cm3_fdb;
	float cm4_fdb;

	float thumbwheel_speed_fdb;               //���̲���תƵ
	float thumbwheel_speed_set;              //���̲���ת��

	uint8_t chassis_control_mode_flag;    //1:б�����˶�    0:��ͨ�˶�
	uint8_t magazine_control_flag;      //1:������      0:�ص���
	uint8_t thumbwheel_move_flag;      //1:������      0:�ص���
	uint8_t thumbwheel_contrary_flag;  //1:����
	uint8_t thumbwheel_remote_flag;    //1:ch4
	uint8_t thumbwheel_acc_flag;       //��תʱ���ۼ�
	uint8_t thumbwheel_forwad_flag;		//������תʱ���ۼ�
	uint8_t thumbwheel_stop_flag;		//ֹͣ������־
	
}chassis_control_data_t;

uint8_t get_robot_control_mode(void);
uint8_t get_robot_work_mode(void);
void magazine_control(chassis_control_data_t *chassis);




typedef struct
{
	float kp;
	float ki;
	float kd;
	
	float ioutput_max;
	float output_max;
	uint8_t mode;
}cali_pid_t;

typedef struct
{
	cali_pid_t cm_pid;
	cali_pid_t rotate_pid;
	cali_pid_t thumbwheel_pid;
	// cascade_pid_t trigger_pid;
}cali_chassis_t;

typedef struct
{
	cali_pid_t position;
	cali_pid_t speed;
}cali_cascade_pid_t;

void chassis_set_and_fdb_update(chassis_control_data_t *chassis,chassis_pid_t *chassis_pid);
void chassis_pid_calculate(chassis_control_data_t *chassis,chassis_pid_t *chassis_pid);
void chassis_task(void *argument);

#endif
