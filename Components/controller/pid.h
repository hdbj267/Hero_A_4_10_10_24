#ifndef PID_H
#define PID_H
#include "main.h"

enum PID_MODE
{
	PID_POSITION = 0,
    PID_DELTA = 1
};

typedef struct pid_t/*????????,?????????*/
{
	float set;
	float fdb;

	float err[3];
	
	float kp;
	float ki;
	float kd;

	float iout;
	float ioutMax;
	
	float output;
	float outputMax;
	
	float kp_offset;
	float ki_offset;
	float kd_offset;
	
	uint8_t mode;
	
	void (*Calc)(struct pid_t *pid);//????
	void (*Reset)(struct pid_t *pid);
}pid_t;

typedef struct
{
	pid_t position_pid;
	pid_t speed_pid;
}cascade_pid_t;

void PID_Calc(pid_t *pid);
void PID_Reset(pid_t *pid);

#endif
