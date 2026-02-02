#include "bsp_usart.h"
#include "main.h"
#include "string.h"
#include "pid.h"
#include "All_struct.h"
#include "Information_task.h"
#include "Gimbal_task.h"
#include "Chassis_task.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

void usart1_tx_dma_init(void)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart1_tx.Instance->NDTR = 0;


}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}



void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);

}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}


/***************************************************************************************/

uint8_t rx_buffer[100];   //接收数据的数组
uint8_t rx_len = 0; //接收数据的长度
uint8_t recv_end_flag = 0; //接收结束标志位



void user_usart_init(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  //开启空闲中断
    HAL_UART_Receive_DMA(&huart1,rx_buffer,100);  //开启DMA接收中断
}

int vision_test_cnt,navigate_cnt;

void usart1_data_process(uint8_t* data,uint8_t len)
{
    if(len==DEBUG_PID_DATA_LEN)//调试用
    {
        if(data[0]==0xA5 && data[18]==0x5A)
        {
            memcpy(&debug_pid,&data[1],len-3);

//            pid_para_realtime_update(&shoot_p[0],&debug_pid);
//            pid_para_realtime_update(&shoot_p[1],&debug_pid);
//                P_K1=debug_pid.Kp;
//                P_K2=debug_pid.Ki;
//                P_a=debug_pid.Kd;
//            Gimbal.Sub_L.yaw_angle_set=(float)debug_pid.set;
/* chassis_v  sub_gimbal_yaw_v  sub_gimbal_yaw_p sub_gimbal_pitch_v*/
/* sub_gimbal_pitch_p  mother_gimbal_yaw_v  mother_gimbal_yaw_p  shoot_v  shoot_p*/
        }
    }
    else if(len==VISION_DATA_LEN)
    {
        if(data[0]==0x5A && data[VISION_DATA_LEN-1]==0x5B)
        {
            memcpy(&Vision,&data[1],VISION_DATA_LEN-2);
            vision_test_cnt++;
            vision_detect_flag=1;
            if(vision_test_cnt>=25)
            {
                HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);
                vision_test_cnt=0;
            }
        }
    }
    else if(len==NAVIGATE_LEN)
    {
        if(data[0]==0xAB && data[NAVIGATE_LEN-1]==0xBA)
        {
            memcpy(&Navigate,&data[1],NAVIGATE_LEN-2);
            navigation_detect_flag=1;
            navigate_cnt++;
            if(navigate_cnt>=25)
            {
                HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
                navigate_cnt=0;
            }
        }
    }
//    else if(len==ALL)
//    {
//        if(data[0]==0x5A && data[VISION_DATA_LEN-1]==0x5B)//视觉
//        {
//            memcpy(&Vision,&data[1],VISION_DATA_LEN-2);
//            vision_test_cnt++;
//            detect_flag=1;
//            if(vision_test_cnt>=25)
//            {
//                HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);
//                vision_test_cnt=0;
//            }
//
//        }
//
//        if(data[VISION_DATA_LEN]==0xAB && data[VISION_DATA_LEN+NAVIGATE_LEN-1]==0xBA)//导航
//        {
//            memcpy(&Navigate,&data[VISION_DATA_LEN+1],NAVIGATE_LEN-2);
//            navigate_cnt++;
//            if(navigate_cnt>=25)
//            {
//                HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
//                navigate_cnt=0;
//            }
//
//        }
//    }


}














