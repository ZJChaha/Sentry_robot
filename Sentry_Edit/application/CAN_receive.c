

#include <math.h>
#include "CAN_receive.h"
#include "Gimbal_task.h"
#include "Shoot_task.h"
#include "cmsis_os.h"
#include "Information_task.h"
#include "main.h"
#include "Chassis_task.h"
#include "dm_motor_drv.h"
#include "bsp_super_cap.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }


motor_measure_t motor_chassis[4];//底盘
motor_measure_t motor_gimbal[5];//云台		
motor_measure_t motor_shoot[2];//扭蛋		
extend_angle_t extend_2006_motor[2];//扭蛋扩展
extend_angle_t extend_mother_yaw_motor;//母云台扩展
		
		
/******************保留备用**********************/
//extend_angle_t extend_sub_L_yaw_motor;
//extend_angle_t extend_sub_L_pitch_motor;
//
//extend_angle_t extend_sub_R_yaw_motor;
//extend_angle_t extend_sub_R_pitch_motor;
/************************************************/				
		
CAN_TxHeaderTypeDef  m_gimbal_shoot_message;
uint8_t              m_gimbal_shoot_data[8];//母云台

CAN_TxHeaderTypeDef  sub_gimbal_message;
uint8_t              sub_gimbal_data[8];//子云台

CAN_TxHeaderTypeDef  chassis_tx_message;
uint8_t              chassis_can_send_data[8];//底盘


uint8_t              rub_wheel_data[8];//摩擦轮
		
void Can_data_init(void)
{

		for(uint8_t i=0;i<8;i++)
		{
            m_gimbal_shoot_data[i]=0;
			sub_gimbal_data[i]=0;
			chassis_can_send_data[i]=0;
            rub_wheel_data[i]=0;
		}

}



void CAN_cmd_gimbal(int16_t mother_yaw, int16_t l_shoot,int16_t r_shoot,int16_t l_yaw,int16_t l_pitch,int16_t r_yaw,int16_t r_pitch)
{
    uint32_t mother_send_mail_box;
    m_gimbal_shoot_message.StdId = 0x1FF;
    m_gimbal_shoot_message.IDE = CAN_ID_STD;
    m_gimbal_shoot_message.RTR = CAN_RTR_DATA;
    m_gimbal_shoot_message.DLC = 0x08;
    m_gimbal_shoot_data[0]=(r_shoot>>8);
    m_gimbal_shoot_data[1]=r_shoot;
    m_gimbal_shoot_data[2]=(l_shoot>>8);
    m_gimbal_shoot_data[3]=l_shoot;
    m_gimbal_shoot_data[4]=(mother_yaw>>8);
    m_gimbal_shoot_data[5]=mother_yaw;

    HAL_CAN_AddTxMessage(&hcan1, &m_gimbal_shoot_message, m_gimbal_shoot_data, &mother_send_mail_box);

    uint32_t sub_send_mail_box;
    sub_gimbal_message.StdId = 0x1FE;
    sub_gimbal_message.IDE = CAN_ID_STD;
    sub_gimbal_message.RTR = CAN_RTR_DATA;
    sub_gimbal_message.DLC = 0x08;
    sub_gimbal_data[0] = (l_yaw >> 8);
    sub_gimbal_data[1] = l_yaw;
    sub_gimbal_data[2] = (l_pitch>>8);
    sub_gimbal_data[3] = l_pitch;
    sub_gimbal_data[4] = (r_yaw >> 8);
    sub_gimbal_data[5] = r_yaw;
    sub_gimbal_data[6] = (r_pitch >> 8);
    sub_gimbal_data[7] = r_pitch;
    HAL_CAN_AddTxMessage(&hcan2, &sub_gimbal_message, sub_gimbal_data, &sub_send_mail_box);


}




void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_send(uint32_t id, uint8_t *data, uint32_t len)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef Header;
    Header.StdId = id;
    Header.IDE = CAN_ID_STD;
    Header.RTR = CAN_RTR_DATA;
    Header.DLC = len;



    HAL_CAN_AddTxMessage(&hcan1, &Header, data, &send_mail_box);
}





uint8_t rx_data[8];
CAN_RxHeaderTypeDef rx_header;
int count=0;

short DM_fdcan_test;
void CAN_Receive_Callback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data);
	if(hcan == &hcan1)
		{
			switch(rx_header.StdId)
			{
				case 0x201:
				{	
					get_motor_measure(&motor_chassis[0],rx_data)//底盘电机（4个）
                    Chassis.motor[0].vel=(float)motor_chassis[0].speed_rpm*0.10472f/19.2032f;//输出轴转速
                    Chassis.motor[0].torque=(float)motor_chassis[0].given_current*6.f/16384.f;//输出轴力矩

				}break;
				case 0x202:
				{
					get_motor_measure(&motor_chassis[1],rx_data)//拆包
                    Chassis.motor[1].vel=(float)motor_chassis[1].speed_rpm*0.10472f/19.2032f;//输出轴转速
                    Chassis.motor[1].torque=(float)motor_chassis[1].given_current*6.f/16384.f;//输出轴力矩

                }break;
				case 0x203:
				{		
					get_motor_measure(&motor_chassis[2],rx_data)//拆包
                    Chassis.motor[2].vel=(float)motor_chassis[2].speed_rpm*0.10472f/19.2032f;//输出轴转速
                    Chassis.motor[2].torque=(float)motor_chassis[2].given_current*6.f/16384.f;//输出轴力矩

                }break;
				case 0x204:
				{		
					get_motor_measure(&motor_chassis[3],rx_data)//拆包
                    Chassis.motor[3].vel=(float)motor_chassis[3].speed_rpm*0.10472f/19.2032f;//输出轴转速
                    Chassis.motor[3].torque=(float)motor_chassis[3].given_current*6.f/16384.f;//输出轴力矩

                }break;
				case 0x205://右
                {
                    get_motor_measure(&motor_shoot[0],rx_data)
                    Extend2006Angle(&extend_2006_motor[0],motor_shoot[0].ecd);//扭蛋电机（2个）
                    Send_motor.Right.motor_speed=((float)motor_shoot[0].speed_rpm/36.0f);
                    Send_motor.Right.motor_angle=extend_2006_motor[0].ecd_angle;
                    Send_motor.Right.motor_current=motor_shoot[0].given_current;
                }break;
                case 0x206://左
                {
                    get_motor_measure(&motor_shoot[1],rx_data)
                    Extend2006Angle(&extend_2006_motor[1],motor_shoot[1].ecd);
                    Send_motor.Left.motor_speed=((float)motor_shoot[1].speed_rpm/36.0f);
                    Send_motor.Left.motor_angle=extend_2006_motor[1].ecd_angle;
                    Send_motor.Left.motor_current=motor_shoot[1].given_current;

                }break;
                case 0x207://母云台
                {
                    get_motor_measure(&motor_gimbal[4],rx_data)
                    Gimbal.Mother.yaw_motor_speed=(float)motor_gimbal[4].speed_rpm;
                    Gimbal.Mother.yaw_motor_ecd=NormalizeAngle(convert(motor_gimbal[4].ecd)-Mother_Yaw_Middle_ecd);
//                    if(Gimbal.Mother.Yaw_Motor_Init_finish==1)
//                    {
//                        Extend6020Angle(&extend_mother_yaw_motor,motor_gimbal[4].ecd);
//                        Gimbal.Mother.yaw_motor_ecd_extend=extend_mother_yaw_motor.ecd_angle;
//                    }
                }break;
                case 0x443:
                {
                    Cap_Data_Receive(rx_data,&cap);
                    power_model.real_power=cap.P_chassis;
                }
                case 0x31:
                {
                    dm_motor_fbdata(&rub_wheel[0],rx_data);
                }
                case 0x32:
                {
                    dm_motor_fbdata(&rub_wheel[1],rx_data);
                }
                case 0x33:
                {
                    dm_motor_fbdata(&rub_wheel[2],rx_data);
                }
                case 0x34:
                {
                    dm_motor_fbdata(&rub_wheel[3],rx_data);
                }


            }
		}
		else if(hcan==&hcan2)//子云台电机（4个）  Chao_IMU(2个)
		{
			switch(rx_header.StdId)
			{
				case 0x205:
				{
					get_motor_measure(&motor_gimbal[0],rx_data)//拆包
					Gimbal.Sub_L.yaw_motor_ecd=NormalizeAngle(convert(motor_gimbal[0].ecd)-Sub_L_Yaw_Zero_angle);//yaw轴电机机械角度
					Gimbal.Sub_L.yaw_motor_speed=(float)motor_gimbal[0].speed_rpm*0.10472f;//yaw轴电机角速度   1 rpm=0.10472 rad/s
                    gimbal_motor_flag[0]=1;//更新检测标志
                }break;
				case 0x206:
				{
					get_motor_measure(&motor_gimbal[1],rx_data)
					Gimbal.Sub_L.pitch_motor_ecd=NormalizeAngle(convert(motor_gimbal[1].ecd)-Sub_L_Pitch_Zero_angle);//pitch轴电机机械角度
					Gimbal.Sub_L.pitch_motor_speed=(float)motor_gimbal[1].speed_rpm;//pitch轴电机角速度
                    gimbal_motor_flag[1]=1;//更新检测标志
                }break;
				case 0x207:
				{
					get_motor_measure(&motor_gimbal[2],rx_data)
					Gimbal.Sub_R.yaw_motor_ecd=NormalizeAngle(convert(motor_gimbal[2].ecd)-Sub_R_Yaw_Zero_angle);//yaw轴电机机械角度
					Gimbal.Sub_R.yaw_motor_speed=(float)motor_gimbal[2].speed_rpm*0.10472f;//yaw轴电机角速度   1 rpm=0.10472 rad/s
                    gimbal_motor_flag[2]=1;//更新检测标志
				}break;
				case 0x208:
				{
					get_motor_measure(&motor_gimbal[3],rx_data)
					Gimbal.Sub_R.pitch_motor_ecd=NormalizeAngle(convert(motor_gimbal[3].ecd)-Sub_R_Pitch_Zero_angle);//pitch轴电机机械角度
					Gimbal.Sub_R.pitch_motor_speed=(float)motor_gimbal[3].speed_rpm;//pitch轴电机角速度
                    gimbal_motor_flag[3]=1;//更新检测标志
				}break;
                case 0x40://Chao_IMU协议  前四字节为Pitch轴角速度，后四字节为Yaw轴角速度
                {
                    IMU.Left.Pitch_Gyro= ByteToFloat(rx_data);
                    IMU.Left.Yaw_Gyro=ByteToFloat(&rx_data[4]);
                    Gimbal.Sub_L.yaw_gyro=IMU.Left.Yaw_Gyro;
                    Gimbal.Sub_L.pitch_gyro=IMU.Left.Pitch_Gyro;
                    Gimbal.Sub_L.yaw_relative_gyro=Gimbal.Sub_L.yaw_gyro-Gimbal.Mother.yaw_gyro;
                    Gimbal.Sub_L.IMU_init_finish=1;

                }break;
                case 0x42://Chao_IMU协议  前四字节为Pitch轴角速度，后四字节为Yaw轴角速度
                {
                    IMU.Right.Pitch_Gyro= ByteToFloat(rx_data);
                    IMU.Right.Yaw_Gyro=ByteToFloat(&rx_data[4]);
                    Gimbal.Sub_R.yaw_gyro=IMU.Right.Yaw_Gyro;
                    Gimbal.Sub_R.pitch_gyro=IMU.Right.Pitch_Gyro;
                    Gimbal.Sub_R.yaw_relative_gyro=Gimbal.Sub_R.yaw_gyro-Gimbal.Mother.yaw_gyro;
                    Gimbal.Sub_R.IMU_init_finish=1;
                }break;
			}
		}

}

void Extend6020Angle(volatile extend_angle_t *motor, uint16_t raw_angle)
{   
	motor->last_raw_value = motor->raw_value;
	motor->raw_value = raw_angle;
	motor->diff = motor->raw_value - motor->last_raw_value;
	if(motor->diff < -6191)  
	{
		motor->round_cnt++;
	}
	else if(motor->diff>6191)
	{
		motor->round_cnt--;
	}		
	motor->ecd_angle = ((float)motor->raw_value/8191.0f + motor->round_cnt)*360.0f-180.0f;      //8191
}


void Extend2006Angle(volatile extend_angle_t *motor, uint16_t raw_angle)
{
    motor->last_raw_value = motor->raw_value;
    motor->raw_value = raw_angle;
    motor->diff = motor->raw_value - motor->last_raw_value;
    if(motor->diff < -6191)
    {
        motor->round_cnt++;
    }
    else if(motor->diff>6191)
    {
        motor->round_cnt--;
    }
    motor->ecd_angle = ((float)motor->raw_value/8191.0f + motor->round_cnt)*10.0f;      //    360.0f/36.0f 2006减速比36:1
}









float convert(uint16_t raw_angle)
{
    float val;
    val=(float)raw_angle*360.0f/8191.0f-180.0f;
    return val;
}

CAN_TxHeaderTypeDef IMU_tx_message;

void Can_Send_IMU_Init(void)
{
/*Chao_IMU第一代，由硬件组吴超设计并生产，应用在两个子云台上*/

/*Chao_IMU协议:  ID:0x41 发送内容:0x4A      ID:0x39 发送内容:0x5A   初始化*/
    for (int i = 0; i < 6; ++i)
    {
        uint32_t send_mail_box_right;
        IMU_tx_message.StdId = 0x41;
        IMU_tx_message.IDE = CAN_ID_STD;
        IMU_tx_message.RTR = CAN_RTR_DATA;
        IMU_tx_message.DLC = 0x08;
        uint8_t data_right[8]={0};
        data_right[0]=0x4A;
        HAL_CAN_AddTxMessage(&hcan2,&IMU_tx_message,data_right,&send_mail_box_right);

        osDelay(2);

        uint32_t send_mail_box_left;
        IMU_tx_message.StdId = 0x39;
        IMU_tx_message.IDE = CAN_ID_STD;
        IMU_tx_message.RTR = CAN_RTR_DATA;
        IMU_tx_message.DLC = 0x08;
        uint8_t data_left[8]={0};
        data_left[0]=0x5A;
        HAL_CAN_AddTxMessage(&hcan2,&IMU_tx_message,data_left,&send_mail_box_left);

        osDelay(5);

    }



}

float ByteToFloat(unsigned char* arr)
{
    return *((float*)arr);
}

float NormalizeAngle(float angle) {
    float a = fmodf(angle + 180.f,360.f);
    if (a < 0.0f) {
        a += (360.f);
    }
    return a - 180.f;
}

