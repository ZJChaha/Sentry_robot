#include "vofa.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "math.h"
#include "usart.h"
#include "All_struct.h"
#include "Chassis_Task.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "remote_control.h"


#include "QuaternionEKF.h"
#define MAX_BUFFER_SIZE 1024
uint8_t send_buf[MAX_BUFFER_SIZE];
uint16_t cnt = 0;

/**
***********************************************************************
* @brief:      vofa_start(void)
* @param:		   void
* @retval:     void
* @details:    �������ݸ���λ��
***********************************************************************
**/
void vofa_start(void)
{
    vofa_demo();		// demoʾ��

//	vofa_sendframetail();
}

/**
***********************************************************************
* @brief:      vofa_transmit(uint8_t* buf, uint16_t len)
* @param:		void
* @retval:     void
* @details:    �޸�ͨ�Ź��ߣ�USART����USB
***********************************************************************
**/
void vofa_transmit(uint8_t* buf, uint16_t len)
{
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len);
    CDC_Transmit_FS((uint8_t *)buf, len);

}
/**
***********************************************************************
* @brief:      vofa_send_data(float data)
* @param[in]:  num: ���ݱ�� data: ����
* @retval:     void
* @details:    ���������ݲ�ֳɵ��ֽ�
***********************************************************************
**/
void vofa_send_data(uint8_t num, float data)
{
    send_buf[cnt++] = byte0(data);
    send_buf[cnt++] = byte1(data);
    send_buf[cnt++] = byte2(data);
    send_buf[cnt++] = byte3(data);
}
/**
***********************************************************************
* @brief      vofa_sendframetail(void)
* @param      NULL
* @retval     void
* @details:   �����ݰ�����֡β
***********************************************************************
**/
void vofa_sendframetail(void)
{
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x80;
    send_buf[cnt++] = 0x7f;


    vofa_transmit((uint8_t *)send_buf, cnt);
    cnt = 0;
}
/**
***********************************************************************
* @brief      vofa_demo(void)
* @param      NULL
* @retval     void
* @details:   demoʾ��
***********************************************************************
**/

#include "referee.h"
#include "bsp_super_cap.h"
#include "Gimbal_task.h"
#include "All_struct.h"
#include "pid.h"
void vofa_demo(void)
{
    /*
    float temp_1=fre_1;
    float temp_2=fre_2;
    vofa_send_data(0,init_speed_1);
    vofa_send_data(1,init_speed_2);
    vofa_send_data(2,temp_1);
    vofa_send_data(3,temp_2);
*/


//    vofa_send_data(0,power_model.real_power);
//    vofa_send_data(1,power_model.power_predict_all);
//    vofa_send_data(1,receive.V_cap);
//    vofa_send_data(2,power_heat_data_t.buffer_energy);



    vofa_send_data(1,Vision.r_yaw);
    vofa_send_data(1,Vision.l_yaw);

    float temp1=Vision.r_flag;
    float temp2=Vision.l_flag;
    vofa_send_data(1,temp1);
    vofa_send_data(1,temp2);

//    vofa_send_data(1,Gimbal.Sub_L.pitch_angle_set);
//    vofa_send_data(1,Gimbal.Sub_L.pitch_motor_ecd);
//    vofa_send_data(1,Gimbal.Sub_R.pitch_motor_ecd);


//    float temp=hurt_data.armor_id;
//    vofa_send_data(1,temp);

    vofa_sendframetail();
}











