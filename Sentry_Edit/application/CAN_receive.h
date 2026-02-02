

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "stdint.h"

#include "can.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2










/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;



 typedef struct{
	uint16_t raw_value;   									
	uint16_t last_raw_value;								                  
	int32_t diff;													
	int32_t round_cnt;										
	float ecd_angle;
}extend_angle_t;

//6020电机控制电流, 范围 [-30000,30000]
//2006电机控制电流, 范围 [-10000,10000]
//3508电机控制电流, 范围 [-16384,16384]


extern motor_measure_t motor_chassis[4];//底盘


extern motor_measure_t motor_gimbal[5];//云台
extern motor_measure_t motor_shoot[2];//扭蛋
extern extend_angle_t extend_2006_motor[2];//扭蛋扩展
extern extend_angle_t extend_mother_yaw_motor;//母云台扩展



 void Can_data_init(void);

void CAN_cmd_gimbal(int16_t mother_yaw, int16_t l_shoot,int16_t r_shoot,int16_t l_yaw,int16_t l_pitch,int16_t r_yaw,int16_t r_pitch);


extern void CAN_cmd_chassis_reset_ID(void);

void CAN_cmd_send(uint32_t id, uint8_t *data, uint32_t len);


extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


void Extend6020Angle(volatile extend_angle_t *motor, uint16_t raw_angle);

void Extend2006Angle(volatile extend_angle_t *motor, uint16_t raw_angle);

void CAN_Receive_Callback(CAN_HandleTypeDef *hcan);

float convert(uint16_t raw_angle);

float ByteToFloat(unsigned char* arr);

void Can_Send_IMU_Init(void);

float NormalizeAngle(float angle);

#endif
