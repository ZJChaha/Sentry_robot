//
// Created by ZJC on 2024/1/24.
//

#ifndef SENTRY_EDIT_DETECT_RIGHT_TASK_H
#define SENTRY_EDIT_DETECT_RIGHT_TASK_H


#include "stdint.h"




extern uint8_t right_smooth_flag;
extern int right_block_cnt;

void Detect_right_task(void const *pvParameters);







#endif //SENTRY_EDIT_DETECT_RIGHT_TASK_H
