
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
//#include "struct_typedef.h"
#include "stdint.h"


#define Chassis_Speed_MAX 2.f //m/s  //1500.0f  //单位: rpm
#define Gimbal_Spin_MAX 6.f
#define Gyro_Speed 6.f  //单位: rad/s
#define Gain_Speed 0.0f

#define SQRT2_HALF 0.70710678f
#define WHEEL_RADIUS 0.076f
#define CHASSIS_RADIUS 0.264f

typedef struct
{
    float pos;//位置
    float last_pos;//上一时刻位置
    float turns;//圈数
    float muti_turns_pos;//多圈位置
    float final_pos;//输出轴端位置
    float vel;//速度
    float torque;//力矩
    float temparature;//温度
    uint8_t err_code;//错误码

    float pos_set;
    float vel_set;
    float torque_set;
    float voltage_set;

}test_motor_t;


typedef struct
{
    float speed_x;
    float speed_y;
    float speed_z;
}__attribute__ ((packed)) Navigate_t;

extern Navigate_t Navigate;

typedef struct
{
float x;
float y; 
float z;
}Speed_para;

typedef struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;


typedef struct
{
float relative_yaw;
float Rotate_speed;
int16_t send_current[4];
first_order_filter_type_t slow_set_vx;
first_order_filter_type_t slow_set_vy;
first_order_filter_type_t slow_set_vz;
Speed_para speed_set;
Speed_para speed_fed;
test_motor_t motor[4];

}Chassis_para;

extern Chassis_para Chassis;


typedef struct
{

    float power_predict[4];//单个电机的预测功率
    float power_limit[4];//单个电机被限制后的功率
    float power_predict_all;//预测总功率
    float power_MAX;//功率限制值
    float K_scale;//缩放系数
    float Torque_limit[4];//最终的输出力矩
    float real_power;//真实力矩

}power_model_t;

extern power_model_t power_model;

extern float P_K1,P_K2,P_a;

void Chassis_task(void const *pvParameters);
void Chassis_Data_update(Chassis_para *para,const uint8_t* mode);
void Chassis_kinematics_calc(Chassis_para *para);
void Can_data_init(void);
//一阶滤波初始化
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
//一阶滤波计算
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
void power_control(power_model_t* para);


#endif
