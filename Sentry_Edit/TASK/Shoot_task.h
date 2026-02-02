#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H
#include "stdint.h"




#define rub_speed 34.f

typedef struct
{
    float speed_set;
    float angle_set;
	float motor_speed;
    float motor_angle;
    float motor_temp;
    short motor_out;
    short motor_current;
	
}motor_para;

typedef struct
{

    motor_para Left;
    motor_para Right;

}Send_motor_para;

extern Send_motor_para Send_motor;

extern uint8_t l_shoot_enable,r_shoot_enable;


void Shoot_task(void const *pvParameters);





void Speed_set(void);



#endif
