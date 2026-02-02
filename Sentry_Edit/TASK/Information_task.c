
#include "Information_task.h"
#include "Chassis_task.h"
#include "Gimbal_task.h"
#include "remote_control.h"
#include "usart.h"
#include "Buzzer_task.h"
#include "cmsis_os.h"
#include "kalman.h"
#include "referee.h"

uint8_t Mode=0,last_Mode=0;

Vision_t Vision;
Flag_t flag;

uint8_t vision_detect_flag,navigation_detect_flag,game_mode;
uint8_t select_mode_finish,Auto_Mode_policy;
uint32_t vision_detect_cnt,navigation_detect_cnt;

uint8_t send_cnt;
Kalman_gimbal_info_t Gimbal_info;
Vision_send_t Vision_send;
Navigation_send_t Navigation_send;




void Information_task(void const *pvParameters)
{

    osDelay(500);


    Navigation_send.head=0X6A;
    Navigation_send.end=0X6B;



    while(!Gimbal_All_Element_Init_Finish)//等待云台初始化完成
    {osDelay(10);}




    if(game_mode==1)
    {
        while(!(game_state.game_progress==3&&game_state.stage_remain_time==0))
        {
            osDelay(10);

            if(RC->rc.s[1]==2)
            {
                SoftReset();
            }

        }
    }




    while(1)
    {
        Set_Mode(&Mode);

        last_Mode=Mode;

#if Vision_detect_mode==1 //miniPC挂机检测
        if(Mode==Auto)
        {
            if(vision_detect_flag==0)
                vision_detect_cnt++;
            else
                vision_detect_cnt=0;
            if(vision_detect_cnt>=10)//若10个周期都没有接收到数据，则认定为视觉挂机
            {
                 Vision.l_flag=0;
                 Vision.r_flag=0;
            }
            vision_detect_flag=0;


            if(navigation_detect_flag==0)
                navigation_detect_cnt++;
            else
                navigation_detect_cnt=0;
            if(navigation_detect_cnt>=100)//若100个周期都没有接收到数据，则认定为导航挂机
            {
                Navigate.speed_y=0;
                Navigate.speed_x=0;
                Navigate.speed_z=0;
            }
            navigation_detect_flag=0;
        }

#endif

        send_cnt++;
        if(send_cnt==10)
        {
            Navigation_info_send();
            send_cnt=0;
        }




        osDelay(10);


    }

}


void Set_Mode(uint8_t* mode)
{
	if(RC->rc.s[0]==0x02)//右下  自动
        *mode=Auto;
    else if( RC->rc.s[0]==0x03)//右中  遥控底盘
        *mode=Remote_Chassis;
    else if( RC->rc.s[0]==0x01)//右上  松
        *mode=Stop;

    if(RC->rc.s[1]==2)
    {
        SoftReset();
    }


}



void Kalman_filter_init(void)
{
    KalmanCreate(&Gimbal_info.Sub_L.Vision_yaw,0.5f,0.5f);
    KalmanCreate(&Gimbal_info.Sub_L.Vision_pitch,0.5f,0.5f);
    KalmanCreate(&Gimbal_info.Sub_L.Vision_distance,0.5f,0.5f);
    KalmanCreate(&Gimbal_info.Sub_R.Vision_yaw,0.5f,0.5f);
    KalmanCreate(&Gimbal_info.Sub_R.Vision_pitch,0.5f,0.5f);
    KalmanCreate(&Gimbal_info.Sub_R.Vision_distance,0.5f,0.5f);

}

int abs_i(int n)
{
    int i;
    i=(n>0) ? n:-n;
    return i;
}

void Navigation_info_send(void)
{

    Navigation_send.robot_id=get_robot_id();
    Navigation_send.gain_zone=(uint8_t)(rfid_status.rfid_status&0x40000);
    Navigation_send.supply_zone=(uint8_t)(rfid_status.rfid_status&0x400000);
    Navigation_send.red_1_robot_HP=game_robot_HP_t.red_1_robot_HP;
    Navigation_send.red_3_robot_HP=game_robot_HP_t.red_3_robot_HP;
    Navigation_send.blue_1_robot_HP=game_robot_HP_t.blue_1_robot_HP;
    Navigation_send.blue_3_robot_HP=game_robot_HP_t.blue_3_robot_HP;

    if(Navigation_send.robot_id==107)
    {
        Navigation_send.red_7_robot_HP=game_robot_HP_t.red_7_robot_HP;
        Navigation_send.blue_7_robot_HP=robot_state.current_HP;
    }
    else if(Navigation_send.robot_id==7)
    {
        Navigation_send.red_7_robot_HP=robot_state.current_HP;
        Navigation_send.blue_7_robot_HP=game_robot_HP_t.blue_7_robot_HP;
    }

    Navigation_send.allow_num=projectile_allowance.projectile_allowance_17mm;

    HAL_UART_Transmit_DMA(&huart1,(uint8_t*)&Navigation_send,19);

}


void SoftReset(void)
{
    __set_FAULTMASK(1);//禁止所有的可屏蔽中断
    NVIC_SystemReset();//软件复位
}


