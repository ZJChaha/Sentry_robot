//
// Created by ZJC on 2024/1/11.
//

#ifndef SENTRY_EDIT_ALL_STRUCT_H
#define SENTRY_EDIT_ALL_STRUCT_H
#include "stdint.h"
typedef struct {
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
    float B;
    float Q;
    float R;
    float H;
}extKalman_t;

typedef struct
{
    uint8_t IMU_init_finish;
    uint8_t Pitch_Motor_Init_finish;
    uint8_t Yaw_Motor_Init_finish;

    float yaw_angle;//陀螺仪角度反馈
    float pitch_angle;

    float yaw_motor_ecd;//电机角度反馈
    float yaw_motor_ecd_extend;//电机角度扩展
    float pitch_motor_ecd;

    float yaw_motor_speed;	//电机速度反馈
    float pitch_motor_speed;

    float yaw_gyro;//陀螺仪yaw轴角速度
    float yaw_relative_gyro;//yaw轴相对角速度(子云台相对母云台)
    float pitch_gyro;

    float yaw_zero_angle;//自身零点

    float yaw_angle_set;
    float yaw_speed_set;
    short yaw_speed_out;
    float yaw_position_out;

    float pitch_angle_set;
    float pitch_speed_set;
    short pitch_speed_out;
    float pitch_position_out;

    float pitch_add;//扫描状态下pitch轴的增量
    float yaw_add;//扫描状态下yaw轴的增量


//    float vision_pitch_err; //pitch轴相对当前值应该偏移的角度
//    float vision_yaw_err;   //yaw轴相对当前值应该偏移的角度
//    float distance;

    float predict_yaw_err;//云台调度预测值

}gimbal_para;

typedef struct
{
    gimbal_para Sub_L;
    gimbal_para Sub_R;
    gimbal_para Mother;
}Gimbal_para;


typedef struct
{
    uint8_t init_scan;
    uint8_t scan;
    uint8_t transiting;
    uint8_t aim;
    uint8_t stop_aim;

}Flag_t;

typedef struct
{
    float Pitch_Gyro;
    float Yaw_Gyro;
}imu_t;


typedef struct
{
    imu_t Left;
    imu_t Right;
}sub_imu_t;

extern sub_imu_t IMU;


typedef struct
{
    //uint8_t head; 0x5A

    uint8_t l_flag;
    float l_yaw;
    float l_pitch;
    float l_distance;

    uint8_t r_flag;
    float r_yaw;
    float r_pitch;
    float r_distance;


    //uint8_t end;  0x5B

}__attribute__ ((packed)) Vision_t;


typedef struct
{
    uint8_t head;
    uint8_t robot_id;
    float r_pitch;
    float l_pitch;
    uint8_t end;
}__attribute__ ((packed)) Vision_send_t;

typedef struct
{
    uint8_t head;//0x6A
    uint8_t robot_id;
    uint8_t gain_zone;
    uint8_t supply_zone;
    uint16_t red_1_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t allow_num;
    uint8_t end;//0x6B
}__attribute__ ((packed)) Navigation_send_t;













extern Vision_t Vision;



#endif //SENTRY_EDIT_ALL_STRUCT_H
