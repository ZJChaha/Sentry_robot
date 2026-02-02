#ifndef DETECT_TASK_H
#define DETECT_TASK_H

#include "stdint.h"




extern uint8_t left_smooth_flag;
extern int left_block_cnt;


void Detect_left_task(void const *pvParameters);




#endif
