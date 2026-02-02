#include "Detect_right_task.h"
#include "cmsis_os.h"
#include "Shoot_task.h"
#include "Buzzer_task.h"
#include "math.h"
#include "remote_control.h"
#include "Gimbal_task.h"
#include "Information_task.h"


int right_block_cnt=0,right_block_flag=0,right_protect_cnt=0;

void Detect_right_task(void const *pvParameters) {

    osDelay(500);
    while (!Gimbal_All_Element_Init_Finish) { osDelay(10); }

    while (RC->rc.s[1] != 1) { osDelay(10); }


    while(1)
    {
        if (RC->rc.s[1] == 1)
        {
            Send_motor.Right.speed_set=120;

            if(Send_motor.Right.motor_current>9800)
                right_block_cnt++;
            else
                right_block_cnt=0;

            if(right_block_cnt>40)
                right_block_flag=1;

            if(right_block_flag==1)
            {
                Send_motor.Right.speed_set=-30;
                right_protect_cnt++;

                if(right_protect_cnt==20)
                {
                    right_block_flag=0;
                    right_protect_cnt=0;
                }
            }
        }
        else
        {
            Send_motor.Right.speed_set=0;
            right_block_flag=0;
            right_protect_cnt=0;
            right_block_cnt=0;
        }

        osDelay(10);
    }
}







