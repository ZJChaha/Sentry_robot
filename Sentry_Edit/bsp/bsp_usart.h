#ifndef BSP_USART_H
#define BSP_USART_H
//#include "struct_typedef.h"
#include "stdint.h"




#define DEBUG_PID_DATA_LEN 19
#define VISION_DATA_LEN 28
#define NAVIGATE_LEN 14
#define ALL 42

extern uint8_t rx_buffer[100];   //接收数据的数组
extern uint8_t rx_len; //接收数据的长度
extern uint8_t recv_end_flag; //接收结束标志位

extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void user_usart_init(void);
void usart1_data_process(uint8_t* data,uint8_t len);
extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
#endif
