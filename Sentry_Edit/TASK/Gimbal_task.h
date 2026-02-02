#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "All_struct.h"

#include "stdint.h"
//母云台,子云台yaw轴逆时针均为正转



#define Mother_Gimbal_Max_err 20

//零点
#define Sub_L_Pitch_Zero_angle (-90.f)
#define Sub_R_Pitch_Zero_angle (150.9f)

#define Sub_L_Yaw_Zero_angle (-33.73f)
#define Sub_R_Yaw_Zero_angle (-151.54f)

#define Mother_Yaw_Middle_ecd (-98.8f)



#define Relative_Max_angle 4.f//pitch软件限位
#define Relative_Min_angle (-31.0f)

#define Sub_L_Yaw_init_ecd 90.0f
#define Sub_R_Yaw_init_ecd (-90.0f)

#define Sub_L_Yaw_start_ecd 102.0f//yaw软件限位
#define Sub_L_Yaw_end_ecd (-75.0f)

#define Sub_R_Yaw_start_ecd (-102.0f)
#define Sub_R_Yaw_end_ecd 75.0f




//#define Critical_angle 12.f




//实际角度
//#define Sub_L_Pitch_MAX_angle 93.0f
//#define Sub_L_Pitch_MIN_angle 65.0f
//
//#define Sub_R_Pitch_MAX_angle (-145.0f)
//#define Sub_R_Pitch_MIN_angle (-171.0f)

#define Sub_Gimbal_Distance 0.26f  //两个子云台的间距,单位(m)



extern uint8_t Gimbal_All_Element_Init_Finish;
extern uint8_t gimbal_motor_flag[4];
extern Gimbal_para Gimbal;
extern uint8_t start_flag;

void Gimbal_task(void const *pvParameters);



void Gimbal_Data_update(Gimbal_para* para,const uint8_t* mode);


void Calc_output(Gimbal_para* para,const uint8_t* mode);
void constrain(float* temp,float Max,float Min);


void Gimbal_Motor_init(void);

float convert(uint16_t raw_angle);

float L_Feedforward_torque(float angle);

float R_Feedforward_torque(float angle);

float rad(float angle);

float angle(float rad);

float protect(float a);

float _const(float data);

float limit_sqrt(float temp);

float limit_acos(float temp);

#endif
