#ifndef BUZZER_TASK_H
#define BUZZER_TASK_H
#include "stdint.h"

extern uint8_t buzzer_flag;

void Buzzer_task(void const *pvParameters);

void buzzer_on(uint16_t arr);

void buzzer_off(void);

int caculate_arr(int set);

void Windows_XP(void);

void part1(void);

void part2(void);

void part3(void);

void Funky_town(void);

void trace(void);

void warning(void);

void di(void);

void DJI(void);
#endif

