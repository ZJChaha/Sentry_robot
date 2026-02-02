#include "Detect_left_task.h"
#include "cmsis_os.h"
#include "Shoot_task.h"
#include "Buzzer_task.h"
#include "math.h"
#include "remote_control.h"
#include "Gimbal_task.h"
#include "Information_task.h"




int left_block_cnt=0,left_block_flag=0,left_protect_cnt=0;

void Detect_left_task(void const *pvParameters)
{

    osDelay(500);
    while (!Gimbal_All_Element_Init_Finish) { osDelay(10); }



    while (RC->rc.s[1] != 1) { osDelay(10); }


    while(1)
    {

            if (RC->rc.s[1] == 1)
            {
                Send_motor.Left.speed_set=120;//拨弹电机转速设置，以120ras/s的转速正转

                if(Send_motor.Left.motor_current>9800)//判断电调反馈电流是否到达阈值
                    left_block_cnt++;//到达阈值后，计数
                else
                    left_block_cnt=0;//小于阈值，则清空计数

                if(left_block_cnt>40)//若超阈值时间超过400ms，则判定为堵转
                    left_block_flag=1;

                if(left_block_flag==1)
                {
                    Send_motor.Left.speed_set=-30;//拨弹电机转速设置，以30ras/s的转速反转
                    left_protect_cnt++;//保护程序计数

                    if(left_protect_cnt==20)//保护程序累计执行200ms后跳出
                    {
                        left_block_flag=0;//清除标志位
                        left_protect_cnt=0;//清空计数
                    }
                }
            }
            else
            {
                Send_motor.Left.speed_set=0;
                left_block_flag=0;
                left_protect_cnt=0;
                left_block_cnt=0;
            }


        osDelay(10);
}


}





