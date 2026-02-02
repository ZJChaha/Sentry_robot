/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
		
		
		
/***************************************************************/
pid_type_def chassis_v[4];
pid_type_def follow_gimbal_v;
		
pid_type_def sub_gimbal_yaw_v[2];
pid_type_def sub_gimbal_yaw_p[2];		
		
pid_type_def sub_gimbal_pitch_v[2];	
pid_type_def sub_gimbal_pitch_p[2];			
		
pid_type_def mother_gimbal_yaw_v;
pid_type_def mother_gimbal_yaw_p;
		
pid_type_def shoot_v[2];
pid_type_def shoot_p[2];


debug_pid_t debug_pid;

/**************************************************************/		
/*注意：因为pid参数为动态调试，所以调试过程中去掉const关键字，调试完后要重新添加！！！！！*/



float chassis_v_pid[3]={0.9f,0.003f,0};
float follow_gimbal_pid[3]={0.16f,0,0.f};

float sub_gimbal_yaw_v_pid[3]={6000.f,1.8f,0.f};
float sub_gimbal_yaw_p_pid[3]={0.23f,0.f,0.f};


float sub_l_gimbal_pitch_v_pid[3]={-2500.f,0.f,0};
float sub_l_gimbal_pitch_p_pid[3]={-0.5f,0,0};

float sub_r_gimbal_pitch_v_pid[3]={-2500.f,0,0};
float sub_r_gimbal_pitch_p_pid[3]={-0.5f,0,0};



float mother_gimbal_yaw_v_pid[3]={53000.0f,0,0};
float mother_gimbal_yaw_p_pid[3]={0.052f,0.f,0.013f};

float shoot_v_pid[3]={90.f,0.2f,0};
float shoot_p_pid[3]={2.8f,0,0};


/*const*/
/**************************************************************/
/*注意：因为pid参数为动态调试，所以调试过程中去掉const关键字，调试完后要重新添加！！！！！*/
		
void PID_init(pid_type_def *pid, uint8_t mode, float PID[3], float max_out, float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}



float PID_calc(pid_type_def *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}


void All_pid_para_init(void)
{
    /***********************************************************************************************************************/
    PID_init(&chassis_v[0],PID_POSITION,chassis_v_pid,CHASSIS_MAX,CHASSIS_IMAX);
    PID_init(&chassis_v[1],PID_POSITION,chassis_v_pid,CHASSIS_MAX,CHASSIS_IMAX);
    PID_init(&chassis_v[2],PID_POSITION,chassis_v_pid,CHASSIS_MAX,CHASSIS_IMAX);
    PID_init(&chassis_v[3],PID_POSITION,chassis_v_pid,CHASSIS_MAX,CHASSIS_IMAX);
    PID_init(&follow_gimbal_v,PID_POSITION,follow_gimbal_pid,FOLLOW_MAX,FOLLOW_IMAX);
    /***********************************************************************************************************************/
    PID_init(&sub_gimbal_yaw_v[0],PID_POSITION,sub_gimbal_yaw_v_pid,SUB_GIMBAL_YAW_V_MAX,SUB_GIMBAL_YAW_V_IMAX);
    PID_init(&sub_gimbal_yaw_v[1],PID_POSITION,sub_gimbal_yaw_v_pid,SUB_GIMBAL_YAW_V_MAX,SUB_GIMBAL_YAW_V_IMAX);
    PID_init(&sub_gimbal_yaw_p[0],PID_POSITION,sub_gimbal_yaw_p_pid,SUB_GIMBAL_YAW_P_MAX,SUB_GIMBAL_YAW_P_IMAX);
    PID_init(&sub_gimbal_yaw_p[1],PID_POSITION,sub_gimbal_yaw_p_pid,SUB_GIMBAL_YAW_P_MAX,SUB_GIMBAL_YAW_P_IMAX);
    /***********************************************************************************************************************/
    PID_init(&sub_gimbal_pitch_v[0],PID_POSITION,sub_l_gimbal_pitch_v_pid,SUB_GIMBAL_PITCH_V_MAX,SUB_GIMBAL_PITCH_V_IMAX);
    PID_init(&sub_gimbal_pitch_v[1],PID_POSITION,sub_r_gimbal_pitch_v_pid,SUB_GIMBAL_PITCH_V_MAX,SUB_GIMBAL_PITCH_V_IMAX);
    PID_init(&sub_gimbal_pitch_p[0],PID_POSITION,sub_l_gimbal_pitch_p_pid,SUB_GIMBAL_PITCH_P_MAX,SUB_GIMBAL_PITCH_P_IMAX);
    PID_init(&sub_gimbal_pitch_p[1],PID_POSITION,sub_r_gimbal_pitch_p_pid,SUB_GIMBAL_PITCH_P_MAX,SUB_GIMBAL_PITCH_P_IMAX);
    /***********************************************************************************************************************/
    PID_init(&mother_gimbal_yaw_v,PID_POSITION,mother_gimbal_yaw_v_pid,MOTHER_GIMBAL_YAW_V_MAX,MOTHER_GIMBAL_YAW_V_IMAX);
    PID_init(&mother_gimbal_yaw_p,PID_POSITION,mother_gimbal_yaw_p_pid,MOTHER_GIMBAL_YAW_P_MAX,MOTHER_GIMBAL_YAW_P_IMAX);
    /***********************************************************************************************************************/
    PID_init(&shoot_v[0],PID_POSITION,shoot_v_pid,SHOOT_V_MAX,SHOOT_V_IMAX);
    PID_init(&shoot_v[1],PID_POSITION,shoot_v_pid,SHOOT_V_MAX,SHOOT_V_IMAX);
    PID_init(&shoot_p[0],PID_POSITION,shoot_p_pid,SHOOT_P_MAX,SHOOT_P_IMAX);
    PID_init(&shoot_p[1],PID_POSITION,shoot_p_pid,SHOOT_P_MAX,SHOOT_P_IMAX);
    /***********************************************************************************************************************/
}	



void pid_para_realtime_update(pid_type_def *pid,debug_pid_t* para)
{
    pid->Kp=para->Kp;
    pid->Ki=para->Ki;
    pid->Kd=para->Kd;

}

