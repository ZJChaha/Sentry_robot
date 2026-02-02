/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb输出错误信息
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "USB_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "bsp_usart.h"
#include "usart.h"
#include "INS_task.h"
#include "BMI088driver.h"
#include "CAN_receive.h"
#include "Chassis_task.h"
#include "Shoot_task.h"
#include "Gimbal_task.h"
#include "Detect_left_task.h"
#include "Detect_right_task.h"
#include "pid.h"
#include "bsp_adc.h"
#include "Buzzer_task.h"
#include "referee.h"
#include "protocol.h"
#include "Super_cap_Task.h"
#include "Information_task.h"
#include "vofa.h"


static void usb_printf(const char *fmt,...);

static uint8_t usb_buf[256];



//引入需要观测的变量
/************************************************/

float chassis_power,chassis_buffer;

/***********************************************/


void USB_task(void const * argument)
{
	
	  osDelay(800);
    MX_USB_DEVICE_Init();

    init_vrefint_reciprocal();


float temp_1,temp_2,temp_3,temp_4,voltage;


//while(!detect_flag)//与minipc握手成功后，DJI
//{ osDelay(10);}
//
//buzzer_flag=6;



    while(1)
    {
        osDelay(10);


        vofa_demo();
//uint8_t test[28];
//        test[0]=0x5A;
//        test[27]=0x5B;
//        memcpy(&test[1],&Vision,26);
//
//        CDC_Transmit_FS(test, 28);


//        get_chassis_power_and_buffer(&chassis_power,&chassis_buffer);

//            temp_1=Send_motor.Left.motor_current;
//            temp_2=Send_motor.Right.motor_current;

//        bluetooth_debug(&temp_1,&temp_2);


        /*单个电机设定值 Chassis.motor_temp  电机转速反馈值 motor_chassis[0].speed_rpm*/
        /*Gimbal.Mother.yaw_speed_set  Gimbal.Mother.yaw_gyro*/
        /*Gimbal.Mother.yaw_angle_set  Gimbal.Mother.yaw_motor_ecd*/
        /*Gimbal.Sub_L.yaw_speed_set   Gimbal.Sub_L.yaw_motor_speed*/
        /*Gimbal.Sub_L.pitch_speed_set  Gimbal.Sub_L.pitch_motor_speed*/
        /*Gimbal.Sub_L.yaw_angle_set     Gimbal.Sub_L.yaw_motor_ecd*/
        /*Gimbal.Sub_L.pitch_angle_set  Gimbal.Sub_L.pitch_motor_ecd*/
        /**/
    }

}

static void usb_printf(const char *fmt,...)
{
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf, fmt, ap);

    va_end(ap);


    CDC_Transmit_FS(usb_buf, len);
}

uint8_t data[11]={0};

void bluetooth_debug(float* set,float* ref)
{

    data[0]=0xA5;
    memcpy(&data[1],set,4);
    memcpy(&data[5],ref,4);

data[9]=data[1]+data[2]+data[3]+data[4]+data[5]+data[6]+data[7]+data[8];

    data[10]=0x5A;
    HAL_UART_Transmit_DMA(&huart1,data,11);
}
