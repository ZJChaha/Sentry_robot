#include "Gimbal_task.h"
#include "Chassis_task.h"
#include "Buzzer_task.h"
#include "Information_task.h"
#include "Shoot_task.h"
#include "All_struct.h"
#include "SerialDebug.h"
#include "bsp_usart.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "cmsis_os.h"
#include "usart.h"
#include "pid.h"
#include "remote_control.h"
#include "referee.h"
#include "arm_math.h"

Gimbal_para Gimbal;

uint8_t Gimbal_All_Element_Init_Finish;

sub_imu_t IMU;
uint8_t gimbal_motor_flag[4];
uint32_t vision_cnt,l_gimbal_cnt,r_gimbal_cnt;//

float middle_distance_l,middle_distance_r;
float middle_angle_err_l,middle_angle_err_r;

float armor_pos[4]={0,90.f,180.f,-90.f};


void Gimbal_task(void const *pvParameters)
{
    osDelay(500);

    while(!Gimbal.Mother.IMU_init_finish)//等待母云台IMU初始化完成
    {osDelay(10);}
    Can_Send_IMU_Init();//给子云台的IMU发送初始化信号
    osDelay(500);
    Gimbal.Sub_L.IMU_init_finish=0;
    Gimbal.Sub_R.IMU_init_finish=0;
    while(!(Gimbal.Sub_L.IMU_init_finish==1 && Gimbal.Sub_R.IMU_init_finish==1))//等待Chao_IMU初始化成功
    { osDelay(10);}


    while(!(gimbal_motor_flag[0]
    &&gimbal_motor_flag[1]
    &&gimbal_motor_flag[2]
    &&gimbal_motor_flag[3]))
    {
        osDelay(10);
    }

    Gimbal_Motor_init();//云台所有电机初始化
    while(1)
    {
        if(RC->rc.ch[4]==660)//下，调试模式
        {
            game_mode=0;
            buzzer_flag=5;
            break;
        }
        else if(RC->rc.ch[4]==-660)//上，比赛模式
        {
            game_mode=1;
            buzzer_flag=3;
            break;
        }

        osDelay(10);
    }


    Gimbal_All_Element_Init_Finish=1;//云台所有元素初始化完成.
    // 以上全部初始化成功后才能运行到这里。


    Gimbal.Mother.yaw_zero_angle=Gimbal.Mother.yaw_angle;
    Gimbal.Mother.yaw_angle_set=Gimbal.Mother.yaw_zero_angle;//确定车身相对零点

    if(game_mode==1)
    {
        while(!(game_state.game_progress==3&&game_state.stage_remain_time==0))
        {
            osDelay(10);
        }
    }



    while(1)
    {

        if(Mode==Stop)//松
        {
            Gimbal_Data_update(&Gimbal,&Mode);
            CAN_cmd_gimbal(0,0,0,0,0,0,0);
        }
        else
        {
            Gimbal_Data_update(&Gimbal,&Mode);

            Calc_output(&Gimbal,&Mode);

           CAN_cmd_gimbal(Gimbal.Mother.yaw_speed_out,Send_motor.Left.motor_out,Send_motor.Right.motor_out,Gimbal.Sub_L.yaw_speed_out,Gimbal.Sub_L.pitch_speed_out,Gimbal.Sub_R.yaw_speed_out,Gimbal.Sub_R.pitch_speed_out);
        }

        osDelay(10);

    }


}



void Gimbal_Motor_init(void)
{
    float value,value_l,value_r;

    Gimbal.Sub_R.yaw_add=Yaw_Scan_Speed;
    Gimbal.Sub_L.yaw_add=-Yaw_Scan_Speed;
    Gimbal.Sub_R.pitch_add=Pitch_Scan_Speed;
    Gimbal.Sub_L.pitch_add=Pitch_Scan_Speed;
    Gimbal.Mother.yaw_angle_set=Gimbal.Mother.yaw_motor_ecd;
    Gimbal.Sub_L.yaw_angle_set=Gimbal.Sub_L.yaw_motor_ecd;
    Gimbal.Sub_R.yaw_angle_set=Gimbal.Sub_R.yaw_motor_ecd;
    Gimbal.Sub_L.pitch_angle_set=Gimbal.Sub_L.pitch_motor_ecd;
    Gimbal.Sub_R.pitch_angle_set=Gimbal.Sub_R.pitch_motor_ecd;

    if(Gimbal.Mother.yaw_motor_ecd<0) value=0.5f;
    else value=-0.5f;

    while(!Gimbal.Mother.Yaw_Motor_Init_finish)//母云台归位
    {
        osDelay(10);
        if (fabsf(Gimbal.Mother.yaw_angle_set - 0) > 1.f)
        {
            Gimbal.Mother.yaw_angle_set += value;
        }
        Gimbal.Mother.yaw_position_out = PID_calc(&mother_gimbal_yaw_p, Gimbal.Mother.yaw_motor_ecd,
                                                  Gimbal.Mother.yaw_angle_set);//归位时反馈值使用电机角度
        Gimbal.Mother.yaw_speed_out = (short) PID_calc(&mother_gimbal_yaw_v, Gimbal.Mother.yaw_gyro,
                                                       Gimbal.Mother.yaw_position_out);

        CAN_cmd_gimbal(Gimbal.Mother.yaw_speed_out, 0, 0, 0, 0, 0, 0);

        if (fabsf(Gimbal.Mother.yaw_motor_ecd - 0) < 5.f) {
            Gimbal.Mother.Yaw_Motor_Init_finish = 1;
        }
    }

#if Gimbal_test_mode==1//子云台归零
//    buzzer_flag=5;

    osDelay(500);

    if(Gimbal.Sub_L.yaw_motor_ecd<Sub_L_Yaw_init_ecd) value_l=0.5f;
    else value_l=-0.5f;
    if(Gimbal.Sub_R.yaw_motor_ecd<Sub_R_Yaw_init_ecd) value_r=0.5f;
    else value_r=-0.5f;

    while(!(Gimbal.Sub_L.Yaw_Motor_Init_finish && Gimbal.Sub_R.Yaw_Motor_Init_finish))//子云台yaw轴电机归位(init)
    {
        osDelay(10);
        if(fabsf(Gimbal.Sub_L.yaw_angle_set-Sub_L_Yaw_init_ecd)>1.f)
        {
            Gimbal.Sub_L.yaw_angle_set += value_l;
        }
        if(fabsf(Gimbal.Sub_R.yaw_angle_set-Sub_R_Yaw_init_ecd)>1.f)
        {
            Gimbal.Sub_R.yaw_angle_set += value_r;
        }
        Gimbal.Mother.yaw_angle_set=0;

        Gimbal.Sub_L.yaw_position_out= PID_calc(&sub_gimbal_yaw_p[0],Gimbal.Sub_L.yaw_motor_ecd,Gimbal.Sub_L.yaw_angle_set);
        Gimbal.Sub_L.yaw_speed_out=(short)PID_calc(&sub_gimbal_yaw_v[0],Gimbal.Sub_L.yaw_gyro,Gimbal.Sub_L.yaw_position_out);
        Gimbal.Sub_R.yaw_position_out= PID_calc(&sub_gimbal_yaw_p[1],Gimbal.Sub_R.yaw_motor_ecd,Gimbal.Sub_R.yaw_angle_set);
        Gimbal.Sub_R.yaw_speed_out=(short)PID_calc(&sub_gimbal_yaw_v[1],Gimbal.Sub_R.yaw_gyro,Gimbal.Sub_R.yaw_position_out);
        Gimbal.Mother.yaw_position_out= PID_calc(&mother_gimbal_yaw_p,Gimbal.Mother.yaw_motor_ecd,Gimbal.Mother.yaw_angle_set);
        Gimbal.Mother.yaw_speed_out=(short)PID_calc(&mother_gimbal_yaw_v,Gimbal.Mother.yaw_gyro,Gimbal.Mother.yaw_position_out);

        CAN_cmd_gimbal(Gimbal.Mother.yaw_speed_out,0,0,Gimbal.Sub_L.yaw_speed_out,Gimbal.Sub_L.pitch_speed_out,Gimbal.Sub_R.yaw_speed_out,Gimbal.Sub_R.pitch_speed_out);


        if(fabsf(Gimbal.Sub_L.yaw_motor_ecd-Sub_L_Yaw_init_ecd)<3.0f)
        {
            Gimbal.Sub_L.Yaw_Motor_Init_finish=1;
        }
        if(fabsf(Gimbal.Sub_R.yaw_motor_ecd-Sub_R_Yaw_init_ecd)<3.0f)
        {
            Gimbal.Sub_R.Yaw_Motor_Init_finish=1;
        }
    }

    buzzer_flag=6;

#else
    osDelay(300);
    buzzer_flag=3;
#endif

}

void Gimbal_Data_update(Gimbal_para* para,const uint8_t* mode)
{
    if(*mode==Remote_Chassis)
    {
        para->Sub_L.pitch_angle_set+=((float)RC->rc.ch[3]/660.f)*0.2f;
        para->Sub_R.pitch_angle_set+=((float)RC->rc.ch[3]/660.f)*0.2f;
        para->Sub_L.yaw_angle_set+=((float)RC->rc.ch[4]/660.f)*0.5f;
        para->Sub_R.yaw_angle_set-=((float)RC->rc.ch[4]/660.f)*0.5f;
        para->Mother.yaw_angle_set-=((float)RC->rc.ch[2]/660.0f)*3.0f;
        //para->Mother.yaw_speed_set=((float)RC->rc.ch[2]/660.0f)*Mother_Gimbal_Gyro_MAX;//调试用
        constrain(&para->Sub_L.pitch_angle_set,Relative_Max_angle,Relative_Min_angle);
        constrain(&para->Sub_R.pitch_angle_set,Relative_Max_angle,Relative_Min_angle);
        constrain(&para->Sub_L.yaw_angle_set,Sub_L_Yaw_start_ecd,Sub_L_Yaw_end_ecd);
        constrain(&para->Sub_R.yaw_angle_set,Sub_R_Yaw_end_ecd,Sub_R_Yaw_start_ecd);
    }
    else if(*mode==Auto)
    {
        float Vz=Navigate.speed_z;
        constrain(&Vz,Gimbal_Spin_MAX,-Gimbal_Spin_MAX);//导航转速限幅


        if(Vision.l_flag==0 && Vision.r_flag==0)
        {

#if NAVIGATE_TEST==1
            para->Sub_L.pitch_angle_set+=((float)RC->rc.ch[3]/660.f)*0.2f;
            para->Sub_R.pitch_angle_set+=((float)RC->rc.ch[3]/660.f)*0.2f;
            para->Sub_L.yaw_angle_set+=((float)RC->rc.ch[4]/660.f)*0.5f;
            para->Sub_R.yaw_angle_set-=((float)RC->rc.ch[4]/660.f)*0.5f;
            //para->Mother.yaw_angle_set-=((float)RC->rc.ch[2]/660.0f)*3.0f;

#else
            vision_cnt++;
            l_gimbal_cnt=0;
            r_gimbal_cnt=0;

            if(vision_cnt>=100)
            {
                buzzer_flag = 0;
                if(robot_state.power_management_gimbal_output==1)
                {
                    para->Sub_L.pitch_angle_set += para->Sub_L.pitch_add;
                    para->Sub_R.pitch_angle_set += para->Sub_R.pitch_add;
                    para->Sub_L.yaw_angle_set += para->Sub_L.yaw_add;
                    para->Sub_R.yaw_angle_set += para->Sub_R.yaw_add;
//                    para->Mother.yaw_angle_set-=((float)RC->rc.ch[2]/660.0f)*2.0f;
                }
                if (para->Sub_L.yaw_angle_set >= Sub_L_Yaw_start_ecd) { para->Sub_L.yaw_add = -Yaw_Scan_Speed; }
                else if (para->Sub_L.yaw_angle_set <= Sub_L_Yaw_end_ecd) { para->Sub_L.yaw_add = Yaw_Scan_Speed; }
                else;

                if (para->Sub_R.yaw_angle_set >= Sub_R_Yaw_end_ecd) { para->Sub_R.yaw_add = -Yaw_Scan_Speed; }
                else if (para->Sub_R.yaw_angle_set <= Sub_R_Yaw_start_ecd) { para->Sub_R.yaw_add = Yaw_Scan_Speed; }
                else;

                if (para->Sub_L.pitch_angle_set >= Relative_Max_angle) { para->Sub_L.pitch_add = -Pitch_Scan_Speed; }
                else if (para->Sub_L.pitch_angle_set <= Relative_Min_angle) { para->Sub_L.pitch_add = Pitch_Scan_Speed; }
                else;

                if (para->Sub_R.pitch_angle_set >= Relative_Max_angle) { para->Sub_R.pitch_add = -Pitch_Scan_Speed; }
                else if (para->Sub_R.pitch_angle_set <= Relative_Min_angle) { para->Sub_R.pitch_add = Pitch_Scan_Speed; }
                else;

            }
//            else
//            {
//
//                para->Sub_L.pitch_angle_set=para->Sub_L.pitch_motor_ecd;
//                para->Sub_R.pitch_angle_set=para->Sub_R.pitch_motor_ecd;
//                para->Sub_L.yaw_angle_set=para->Sub_L.yaw_motor_ecd;
//                para->Sub_R.yaw_angle_set=para->Sub_R.yaw_motor_ecd;
//
//            }
#endif

            if(robot_state.power_management_gimbal_output==1)
            {
                if(Vz!=0.f)
                {
                    para->Mother.yaw_angle_set+=Vz*0.01f*57.29578f;
                }
                else
                {
                    if (hurt_receive_flag==1&&hurt_data.HP_deduction_reason==0)//受击检测
                    {
                        float err=armor_pos[hurt_data.armor_id]-Gimbal.Mother.yaw_motor_ecd;
                        if(err>180.f) err-=360;
                        else if(err<-180.f) err+=360;

                        para->Mother.yaw_angle_set=para->Mother.yaw_angle+err;
                        para->Sub_L.yaw_angle_set=Sub_L_Yaw_start_ecd;
                        para->Sub_R.yaw_angle_set=Sub_R_Yaw_start_ecd;
                        para->Sub_L.pitch_angle_set=-10.f;
                        para->Sub_R.pitch_angle_set=-10.f;
                        vision_cnt=0;
                        hurt_receive_flag=0;
                    }
                }
            }
            else
            {
                para->Mother.yaw_angle_set=para->Mother.yaw_angle;
            }
            constrain(&para->Sub_L.pitch_angle_set, Relative_Max_angle, Relative_Min_angle);
            constrain(&para->Sub_R.pitch_angle_set, Relative_Max_angle, Relative_Min_angle);
            constrain(&para->Sub_L.yaw_angle_set, Sub_L_Yaw_start_ecd, Sub_L_Yaw_end_ecd);
            constrain(&para->Sub_R.yaw_angle_set, Sub_R_Yaw_end_ecd, Sub_R_Yaw_start_ecd);
        }
        else if(Vision.l_flag!=0 && Vision.r_flag!=0)
        {
            vision_cnt=0;
            l_gimbal_cnt=0;
            r_gimbal_cnt=0;


            if(Vz==0)
            {
                middle_distance_l=sqrtf(limit_sqrt(powf(Vision.l_distance,2)+powf(Sub_Gimbal_Distance/2,2)-2*arm_cos_f32(rad(180.f-para->Sub_L.yaw_motor_ecd))*Vision.l_distance*(Sub_Gimbal_Distance/2)));
                middle_distance_r=sqrtf(limit_sqrt(powf(Vision.r_distance,2)+powf(Sub_Gimbal_Distance/2,2)-2*arm_cos_f32(rad(180.f+para->Sub_R.yaw_motor_ecd))*Vision.r_distance*(Sub_Gimbal_Distance/2)));
                middle_angle_err_l=angle(acosf(limit_acos((powf(Sub_Gimbal_Distance/2,2)+ powf(middle_distance_l,2)-powf(Vision.l_distance,2))/(middle_distance_l*Sub_Gimbal_Distance))));
                middle_angle_err_r=180.f-angle(acosf(limit_acos((powf(Sub_Gimbal_Distance/2,2)+ powf(middle_distance_r,2)-powf(Vision.r_distance,2))/(middle_distance_r*Sub_Gimbal_Distance))));
                para->Mother.predict_yaw_err=90.f-(middle_angle_err_l+middle_angle_err_r)/2;
                constrain(&para->Mother.predict_yaw_err,Mother_Gimbal_Max_err,-Mother_Gimbal_Max_err);
                para->Mother.yaw_angle_set=para->Mother.yaw_angle-para->Mother.predict_yaw_err;
            }
            else
            {
                para->Mother.yaw_angle_set+=Vz*0.01f*57.29578f;
            }

            para->Sub_L.pitch_angle_set = para->Sub_L.pitch_motor_ecd + Vision.l_pitch;
            para->Sub_L.yaw_angle_set = para->Sub_L.yaw_motor_ecd + Vision.l_yaw;
            para->Sub_R.pitch_angle_set = para->Sub_R.pitch_motor_ecd + Vision.r_pitch;
            para->Sub_R.yaw_angle_set = para->Sub_R.yaw_motor_ecd + Vision.r_yaw;


            constrain(&para->Sub_L.pitch_angle_set,Relative_Max_angle,Relative_Min_angle);
            constrain(&para->Sub_R.pitch_angle_set,Relative_Max_angle,Relative_Min_angle);
            constrain(&para->Sub_L.yaw_angle_set,Sub_L_Yaw_start_ecd,Sub_L_Yaw_end_ecd);
            constrain(&para->Sub_R.yaw_angle_set,Sub_R_Yaw_end_ecd,Sub_R_Yaw_start_ecd);
        }
        else if(Vision.l_flag!=0 && Vision.r_flag==0)
        {

            vision_cnt = 0;

            l_gimbal_cnt++;
            if(l_gimbal_cnt>2)
            {
                r_gimbal_cnt = 0;

                para->Sub_L.pitch_angle_set = para->Sub_L.pitch_motor_ecd + Vision.l_pitch;
                para->Sub_L.yaw_angle_set = para->Sub_L.yaw_motor_ecd + Vision.l_yaw;

                if(Vz==0)
                {
                    para->Sub_R.predict_yaw_err =
                            90.f - angle(acosf(limit_acos(Sub_Gimbal_Distance / (2 * Vision.l_distance))));
                    para->Sub_R.yaw_angle_set = Sub_R_Yaw_init_ecd - para->Sub_R.predict_yaw_err;
                    para->Sub_R.pitch_angle_set = para->Sub_L.pitch_angle_set;

                    middle_distance_l = sqrtf(limit_sqrt(powf(Vision.l_distance, 2) + powf(Sub_Gimbal_Distance / 2, 2) -
                                                         2 * arm_cos_f32(rad(180.f - para->Sub_L.yaw_motor_ecd)) *
                                                         Vision.l_distance * (Sub_Gimbal_Distance / 2)));
                    middle_angle_err_l = angle(acosf(limit_acos((powf(Sub_Gimbal_Distance / 2, 2) + powf(middle_distance_l, 2) -
                                                                 powf(Vision.l_distance, 2)) /
                                                                (middle_distance_l * Sub_Gimbal_Distance))));
                    para->Mother.predict_yaw_err = 90.f - middle_angle_err_l;
                    constrain(&para->Mother.predict_yaw_err, Mother_Gimbal_Max_err, -Mother_Gimbal_Max_err);
                    para->Mother.yaw_angle_set = para->Mother.yaw_angle - para->Mother.predict_yaw_err;
                }
                else
                {
                    if(robot_state.power_management_gimbal_output==1)
                    {
                        para->Mother.yaw_angle_set+=Vz*0.01f*57.29578f;
                        para->Sub_R.pitch_angle_set += para->Sub_R.pitch_add;
                        if (para->Sub_R.pitch_angle_set >= Relative_Max_angle) { para->Sub_R.pitch_add = -Pitch_Scan_Speed; }
                        else if (para->Sub_R.pitch_angle_set <= Relative_Min_angle) { para->Sub_R.pitch_add = Pitch_Scan_Speed; }
                        else;
                        para->Sub_R.yaw_angle_set += para->Sub_R.yaw_add;
                        if (para->Sub_R.yaw_angle_set >= Sub_R_Yaw_end_ecd) { para->Sub_R.yaw_add = -Yaw_Scan_Speed; }
                        else if (para->Sub_R.yaw_angle_set <= Sub_R_Yaw_start_ecd) { para->Sub_R.yaw_add = Yaw_Scan_Speed; }
                        else;
                    }
                    else
                    {
                        para->Mother.yaw_angle_set=para->Mother.yaw_angle;
                        para->Sub_R.pitch_angle_set=para->Sub_R.pitch_motor_ecd;
                        para->Sub_R.yaw_angle_set=para->Sub_R.yaw_motor_ecd;
                    }
                }
            }

            constrain(&para->Sub_L.pitch_angle_set, Relative_Max_angle, Relative_Min_angle);
            constrain(&para->Sub_R.pitch_angle_set, Relative_Max_angle, Relative_Min_angle);
            constrain(&para->Sub_L.yaw_angle_set, Sub_L_Yaw_start_ecd, Sub_L_Yaw_end_ecd);
            constrain(&para->Sub_R.yaw_angle_set, Sub_R_Yaw_end_ecd, Sub_R_Yaw_start_ecd);

        }
        else if(Vision.l_flag==0 && Vision.r_flag!=0)//若左子云台识别到目标，右子云台没有识别到目标，此时进行集火
        {
            vision_cnt = 0;
            r_gimbal_cnt++;
            if(r_gimbal_cnt>2)//相机发现目标超过20ms，进入集火算法
            {
                l_gimbal_cnt = 0;

                para->Sub_R.pitch_angle_set = para->Sub_R.pitch_motor_ecd + Vision.r_pitch;//pitch轴按照上位机发来的角度进行赋值
                para->Sub_R.yaw_angle_set = para->Sub_R.yaw_motor_ecd + Vision.r_yaw;//yaw轴按照上位机发来的角度进行赋值
                if(Vz==0)//母云台转速为0时进入
                {
                    para->Sub_L.predict_yaw_err = angle(acosf(limit_acos(Sub_Gimbal_Distance / (2 * Vision.r_distance)))) - 90.f;
                    //计算左子云台yaw轴预测角度差值
                    para->Sub_L.yaw_angle_set = Sub_L_Yaw_init_ecd - para->Sub_L.predict_yaw_err;//将差值赋给设定角度

                    para->Sub_L.pitch_angle_set = para->Sub_R.pitch_angle_set;//集火过程中，另一个子云台的pitch轴直接使用该云台的角度

                    middle_distance_r = sqrtf(limit_sqrt(powf(Vision.r_distance, 2) + powf(Sub_Gimbal_Distance / 2, 2) -
                                                         2 * arm_cos_f32(rad(180.f + para->Sub_R.yaw_motor_ecd)) *
                                                         Vision.r_distance * (Sub_Gimbal_Distance / 2)));//计算母云台到目标的距离
                    middle_angle_err_r = 180.f - angle(acosf(limit_acos(
                            (powf(Sub_Gimbal_Distance / 2, 2) + powf(middle_distance_r, 2) -
                             powf(Vision.r_distance, 2)) / (middle_distance_r * Sub_Gimbal_Distance))));//计算母云台预测角度

                    para->Mother.predict_yaw_err = 90.f - middle_angle_err_r;//赋值
                    constrain(&para->Mother.predict_yaw_err, Mother_Gimbal_Max_err, -Mother_Gimbal_Max_err);//限幅
                    para->Mother.yaw_angle_set = para->Mother.yaw_angle - para->Mother.predict_yaw_err;
                }
                else
                {
                    if(robot_state.power_management_gimbal_output==1)
                    {
                        para->Mother.yaw_angle_set+=Vz*0.01f*57.29578f;
                        para->Sub_L.pitch_angle_set += para->Sub_L.pitch_add;
                        para->Sub_L.yaw_angle_set += para->Sub_L.yaw_add;
                        if (para->Sub_L.pitch_angle_set >= Relative_Max_angle) { para->Sub_L.pitch_add = -Pitch_Scan_Speed; }
                        else if (para->Sub_L.pitch_angle_set <= Relative_Min_angle) { para->Sub_L.pitch_add = Pitch_Scan_Speed; }
                        else;
                        if (para->Sub_L.yaw_angle_set >= Sub_L_Yaw_start_ecd) { para->Sub_L.yaw_add = -Yaw_Scan_Speed; }
                        else if (para->Sub_L.yaw_angle_set <= Sub_L_Yaw_end_ecd) { para->Sub_L.yaw_add = Yaw_Scan_Speed; }
                        else;
                    }
                    else
                    {
                        para->Mother.yaw_angle_set=para->Mother.yaw_angle;
                        para->Sub_L.pitch_angle_set=para->Sub_L.pitch_motor_ecd;
                        para->Sub_L.yaw_angle_set=para->Sub_L.yaw_motor_ecd;
                    }
                }
            }

            constrain(&para->Sub_L.pitch_angle_set, Relative_Max_angle, Relative_Min_angle);
            constrain(&para->Sub_R.pitch_angle_set, Relative_Max_angle, Relative_Min_angle);
            constrain(&para->Sub_L.yaw_angle_set, Sub_L_Yaw_start_ecd, Sub_L_Yaw_end_ecd);
            constrain(&para->Sub_R.yaw_angle_set, Sub_R_Yaw_end_ecd, Sub_R_Yaw_start_ecd);
        }


    }
    else if(*mode==Stop)
    {
        para->Mother.yaw_angle_set=para->Mother.yaw_angle;
        para->Sub_L.pitch_angle_set=para->Sub_L.pitch_motor_ecd;
        para->Sub_R.pitch_angle_set=para->Sub_R.pitch_motor_ecd;
        para->Sub_L.yaw_angle_set=para->Sub_L.yaw_motor_ecd;
        para->Sub_R.yaw_angle_set=para->Sub_R.yaw_motor_ecd;
    }

}

void Calc_output(Gimbal_para* para,const uint8_t* mode)
{
    if (*mode==Remote_Chassis||*mode==Auto)
    {
        para->Mother.yaw_position_out=PID_calc(&mother_gimbal_yaw_p,para->Mother.yaw_angle,para->Mother.yaw_angle_set);
        para->Mother.yaw_speed_out=(short)PID_calc(&mother_gimbal_yaw_v,para->Mother.yaw_gyro,para->Mother.yaw_position_out);

        para->Sub_L.yaw_position_out= PID_calc(&sub_gimbal_yaw_p[0],para->Sub_L.yaw_motor_ecd,para->Sub_L.yaw_angle_set);
        para->Sub_L.yaw_speed_out=(short)PID_calc(&sub_gimbal_yaw_v[0],para->Sub_L.yaw_relative_gyro,para->Sub_L.yaw_position_out);
        para->Sub_R.yaw_position_out= PID_calc(&sub_gimbal_yaw_p[1],para->Sub_R.yaw_motor_ecd,para->Sub_R.yaw_angle_set);
        para->Sub_R.yaw_speed_out=(short)PID_calc(&sub_gimbal_yaw_v[1],para->Sub_R.yaw_relative_gyro,para->Sub_R.yaw_position_out);

        para->Sub_L.pitch_position_out= PID_calc(&sub_gimbal_pitch_p[0],para->Sub_L.pitch_motor_ecd,para->Sub_L.pitch_angle_set);
        para->Sub_R.pitch_position_out= PID_calc(&sub_gimbal_pitch_p[1],para->Sub_R.pitch_motor_ecd,para->Sub_R.pitch_angle_set);

        para->Sub_L.pitch_speed_out=(short)PID_calc(&sub_gimbal_pitch_v[0],para->Sub_L.pitch_gyro,para->Sub_L.pitch_position_out);
        para->Sub_R.pitch_speed_out=(short)PID_calc(&sub_gimbal_pitch_v[1],para->Sub_R.pitch_gyro,para->Sub_R.pitch_position_out);

//        para->Sub_L.pitch_speed_out=(short)(L_Feedforward_torque(para->Sub_L.pitch_motor_ecd));//测重力补偿用
//        para->Sub_R.pitch_speed_out=(short)(R_Feedforward_torque(para->Sub_R.pitch_motor_ecd));
//
    }

}


void constrain(float* temp,float Max,float Min)
{
    *temp = *temp>=Max?Max:*temp;
    *temp = *temp<=Min?Min:*temp;
}

float limit_sqrt(float temp)
{
    if(temp<0)
        temp=0.000000000001f;

    return temp;
}

float limit_acos(float temp)
{
    if(temp>1)
    {temp=1;}
    else if(temp<-1)
    {temp=-1;}

    return temp;
}

float L_Feedforward_torque(float angle)
{
    float output;

    output= 5800.f*arm_cos_f32(rad(angle));


//    if(angle<-29.44f) {output = 1700.f;}
//    else if(angle>=-29.44f && angle<-28.38f) {output=3783.f*(angle-(-29.44f))+1700.f;}
//    else if(angle>=-28.38f && angle<-27.77f) {output=5678.f;}
//    else if(angle>=-27.77f && angle<-2.f) {output=90.f*(angle-(-27.77f))+5678.f;}
//    else if(angle>=-2.f) {output=8000.f-300.f*(angle-(-2.f));}

        return output;
}

float R_Feedforward_torque(float angle)
{
    float output;

    output= 6000.f*arm_cos_f32(rad(angle));

//    if(angle<-29.4f) {output = 1700.f;}
//    else if(angle>=-29.4f && angle<-28.61f) {output=5686.f*(angle-(-29.4f))+1700.f;}
//    else if(angle>=-28.61f && angle<-15.42f) {output=136.4f*(angle-(-28.61f))+6200.f;}
//    else if(angle>=-15.42f && angle<-8.65f) {output=-47.26f*(angle-(-15.42f))+7720.f;}
//    else if(angle>=-8.65f) {output=57.8f*(angle-(-8.65f))+7400.f;}


    return output;


}


float rad(float angle)
{
    return angle/57.29578f;
}

float angle(float rad)
{
    return rad*57.29578f;
}

float protect(float a)
{
    if(a<=0)
    {
        a=0;
    }
    return a;
}

float _const(float data)
{
    if(data<0)
    {data=0;}
    else if(data>1)
    {data=1;}

}






