#ifndef BSP_CAN_H
#define BSP_CAN_H
//#include "struct_typedef.h"
#include "stdint.h"
#include "can.h"

extern void can_filter_init(void);
void can_Filter_and_it_Init(void);
void HAL_CAN_FilterInit(CAN_HandleTypeDef *hcan);
#endif
