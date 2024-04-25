#include "pid.h"
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note           pid模式选择
  */
void PID_Calc(pid_t *pid)
{
	pid->err[2] = pid->err[1];
	pid->err[1] = pid->err[0];
	pid->err[0] = pid->set - pid->fdb;
	if(pid->mode==PID_DELTA)//???PID	///增量式Pid
	{
		pid->iout = pid->ki*pid->err[0];
		
		pid->output += pid->kp*(pid->err[0]-pid->err[1]) \
					 + pid->iout                         \
					 + pid->kd*(pid->err[0]-2.0f*pid->err[1]+pid->err[2]);
	}
	else if(pid->mode==PID_POSITION)//???PID///位置式pid
	{
		pid->iout += pid->err[0];
		if(pid->iout > pid->ioutMax)            pid->iout = pid->ioutMax;
	    else if(pid->iout < (-pid->ioutMax))    pid->iout = (-pid->ioutMax);
		
		pid->output  = pid->kp*(pid->err[0]) \
					 + pid->ki*pid->iout             \
					 + pid->kd*(pid->err[0]-pid->err[1]);
	}
	//????
	if(pid->output > pid->outputMax)            pid->output = pid->outputMax;
	else if(pid->output < (-pid->outputMax))    pid->output = (-pid->outputMax);
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  */
void PID_Reset(pid_t *pid)
{
	pid->set = 0.0f;
	pid->fdb = 0.0f;
	pid->err[0] = 0.0f;
	pid->err[1] = 0.0f;
	pid->err[2] = 0.0f;
	pid->iout = 0.0f;
	pid->output = 0.0f;
}
