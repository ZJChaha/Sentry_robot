/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
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
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "stdint.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};



#define CHASSIS_MAX 5.f
#define CHASSIS_IMAX 0.7f

#define FOLLOW_MAX 5.0f
#define FOLLOW_IMAX 0.0f

#define SUB_GIMBAL_YAW_V_MAX 16000
#define SUB_GIMBAL_YAW_V_IMAX 4000
#define SUB_GIMBAL_YAW_P_MAX 10
#define SUB_GIMBAL_YAW_P_IMAX 0

#define SUB_GIMBAL_PITCH_V_MAX 16000
#define SUB_GIMBAL_PITCH_V_IMAX 3000
#define SUB_GIMBAL_PITCH_P_MAX 10
#define SUB_GIMBAL_PITCH_P_IMAX 0

#define MOTHER_GIMBAL_YAW_V_MAX 28000
#define MOTHER_GIMBAL_YAW_V_IMAX 5000
#define MOTHER_GIMBAL_YAW_P_MAX  14.0f
#define MOTHER_GIMBAL_YAW_P_IMAX 2.0f

#define SHOOT_V_MAX 10000
#define SHOOT_V_IMAX 2000
#define SHOOT_P_MAX 1000
#define SHOOT_P_IMAX 200


typedef struct
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float set;
}__attribute__((packed)) debug_pid_t;






extern pid_type_def chassis_v[4];
extern pid_type_def follow_gimbal_v;
		
extern pid_type_def sub_gimbal_yaw_v[2];
extern pid_type_def sub_gimbal_yaw_p[2];		
		
extern pid_type_def sub_gimbal_pitch_v[2];	
extern pid_type_def sub_gimbal_pitch_p[2];			
		
extern pid_type_def mother_gimbal_yaw_v;
extern pid_type_def mother_gimbal_yaw_p;
		
extern pid_type_def shoot_v[2];
extern pid_type_def shoot_p[2];

extern debug_pid_t debug_pid;

extern  float chassis_v_pid[3];
extern  float follow_gimbal_pid[3];

extern float sub_gimbal_yaw_v_pid[3];
extern float sub_gimbal_yaw_p_pid[3];

extern float sub_l_gimbal_pitch_v_pid[3];
extern float sub_l_gimbal_pitch_p_pid[3];

extern float sub_r_gimbal_pitch_v_pid[3];
extern float sub_r_gimbal_pitch_p_pid[3];

extern float mother_gimbal_yaw_v_pid[3];
extern float mother_gimbal_yaw_p_pid[3];

extern float	shoot_v_pid[3];
extern float	shoot_p_pid[3];










extern void PID_init(pid_type_def *pid, uint8_t mode, float PID[3], float max_out, float max_iout);


extern float PID_calc(pid_type_def *pid, float ref, float set);


extern void PID_clear(pid_type_def *pid);



void All_pid_para_init(void);


void pid_para_realtime_update(pid_type_def *pid,debug_pid_t* para);














#endif
