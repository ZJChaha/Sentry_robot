//
// Created by ZJC on 2025/2/16.
//
#include "Super_cap_Task.h"
#include "Chassis_Task.h"
#include "cmsis_os.h"
#include "All_struct.h"
#include "bsp_can.h"
#include "bsp_super_cap.h"
#include "CAN_receive.h"
#include "referee.h"


void Super_cap_Task(void const * argument)
{

    osDelay(500);




    for (;;)
    {

        control.P_in = 9000; //限制功率90W   x100
        control.buffer = power_heat_data_t.buffer_energy; //当前缓冲能量 60J
        control.cap_mode = CAP_MODE_AUTOMATIC; //自动模式



        CAN_cmd_send(0x334,(uint8_t*)&control,8);

        osDelay(20);
    }

}


