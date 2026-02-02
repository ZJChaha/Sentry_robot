#include "Shoot_task.h"
#include "Gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "bsp_fric.h"
#include "Information_task.h"
#include "cmsis_os.h"
#include "dm_motor_drv.h"
#include "referee.h"


Send_motor_para Send_motor;
uint8_t l_shoot_enable,r_shoot_enable;
uint32_t message_cnt,fire_cnt;

uint8_t index_rub=0,cnt_rub=0;

void Shoot_task(void const *pvParameters)
{
    osDelay(500);
    dm_motor_init();
    enable_all_motor_os();


    while(!Gimbal_All_Element_Init_Finish)
    {osDelay(10);}

    if(game_mode==1)
    {
        while(!(game_state.game_progress==3&&game_state.stage_remain_time==0))
        {
            osDelay(10);
        }
    }


    while(1)
    {
        message_cnt++;
        if(RC->rc.s[1]==1)//遥控器拨杆开关控制开火
        {
            if(message_cnt>=20)//每个200ms给四个摩擦轮电机发送速度指令
            {
                spd_ctrl(&hcan1,0x21,rub_speed);
                spd_ctrl(&hcan1,0x22,-rub_speed);
                osDelay(1);
                spd_ctrl(&hcan1,0x23,rub_speed);
                spd_ctrl(&hcan1,0x24,-rub_speed);
                message_cnt=0;//清空计数
            }
            if(Mode==Auto)//自动模式识别到自动开火
            {
                if(Vision.r_flag==1)//右边相机发现目标，拨弹电机进入速度闭环模式，按照卡弹保护算法的逻辑进行赋值
                    Send_motor.Left.motor_out=(short)PID_calc(&shoot_v[1],Send_motor.Left.motor_speed,Send_motor.Left.speed_set);//(speed/30)*1.185==射频
                else//右边相机没有识别到目标，速度赋为0
                    Send_motor.Left.motor_out=0;

                if(Vision.l_flag==1)//左边相机发现目标，拨弹电机进入速度闭环模式，按照卡弹保护算法的逻辑进行赋值
                    Send_motor.Right.motor_out=(short)PID_calc(&shoot_v[0],Send_motor.Right.motor_speed,Send_motor.Right.speed_set);
                else//左边相机没有识别到目标，速度赋为0
                    Send_motor.Right.motor_out=0;
            }
            else if(Mode==Remote_Chassis)//手动模式直接开火
            {
                fire_cnt++;
                if(fire_cnt>100)
                {
                    Send_motor.Left.motor_out=(short)PID_calc(&shoot_v[1],Send_motor.Left.motor_speed,Send_motor.Left.speed_set);//(speed/30)*1.185==射频
                    Send_motor.Right.motor_out=(short)PID_calc(&shoot_v[0],Send_motor.Right.motor_speed,Send_motor.Right.speed_set);
                }

            }

        }
        else
        {
            if(message_cnt>=20)
            {
                spd_ctrl(&hcan1,0x21,0.f);
                spd_ctrl(&hcan1,0x22,0.f);
                osDelay(1);
                spd_ctrl(&hcan1,0x23,0.f);
                spd_ctrl(&hcan1,0x24,0.f);
                message_cnt=0;
            }

            Send_motor.Left.motor_out=0;
            Send_motor.Right.motor_out=0;
            fire_cnt=0;
        }

        osDelay(7);
        cnt_rub++;

        if(cnt_rub==24)
        {
            enable_motor_mode(&hcan1,0x21+index_rub,SPD_MODE);
            index_rub++;
            if(index_rub==4) index_rub=0;

            cnt_rub=0;
        }
        osDelay(1);



    }

}













