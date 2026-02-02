#include "Chassis_task.h"
#include "Gimbal_task.h"
#include "Buzzer_task.h"
#include "Information_task.h"
#include "Can_receive.h"
#include "remote_control.h"
#include "bsp_super_cap.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include "arm_math.h"
#include "pid.h"
#include "referee.h"

Chassis_para Chassis;
Navigate_t Navigate;
power_model_t power_model;
uint32_t gyro_cnt,position_cnt;
uint8_t first_gyro_flag;

float chassis_x_order_filter[1] = {0.3f};
float chassis_y_order_filter[1] = {0.3f};
float chassis_z_order_filter[1] = {0.3f};
float P_K1=0.018f,P_K2=1.22f,P_a=0.55f;

void Chassis_task(void const *pvParameters)
{
    osDelay(500);

    first_order_filter_init(&Chassis.slow_set_vx,0.01f,chassis_x_order_filter);
    first_order_filter_init(&Chassis.slow_set_vy,0.01f,chassis_y_order_filter);
    first_order_filter_init(&Chassis.slow_set_vz,0.01f,chassis_z_order_filter);
    while(!Gimbal_All_Element_Init_Finish)
    {osDelay(10);}

    while(RC->rc.s[1]!=3)
    {
        osDelay(10);
    }

    if(game_mode==1)
    {
        while(!(game_state.game_progress==3&&game_state.stage_remain_time==0))
        {
            osDelay(10);
        }
    }



    while(1)
    {		


        if(Mode==Stop)
            CAN_cmd_chassis(0,0,0,0);
        else
        {
            Chassis_Data_update(&Chassis,&Mode);
            Chassis_kinematics_calc(&Chassis);

            for(uint8_t i=0;i<4;i++)
            {
                Chassis.motor[i].torque_set=PID_calc(&chassis_v[i],Chassis.motor[i].vel,Chassis.motor[i].vel_set);

            }

            power_control(&power_model);

            for (int i = 0; i < 4; ++i)
            {
              Chassis.send_current[i]=(short)(power_model.Torque_limit[i]*2730.666f);     //1*16384/(20*0.3)
//              Chassis.send_current[i]=(short)(Chassis.motor[i].torque_set*2730.666f);     //1*16384/(20*0.3)
            }


            CAN_cmd_chassis(Chassis.send_current[0],Chassis.send_current[1],Chassis.send_current[2],Chassis.send_current[3]);
        }

        osDelay(10);
			
    }
}

	
void Chassis_Data_update(Chassis_para *para,const uint8_t* mode)
{
    para->relative_yaw=(0-Gimbal.Mother.yaw_motor_ecd)/57.29578f;

	if(*mode==Remote_Chassis)
	{

//        para->speed_set.x=((float)RC->rc.ch[0]/660.0f)*Chassis_Speed_MAX;
//        para->speed_set.y=((float)RC->rc.ch[1]/660.0f)*Chassis_Speed_MAX;

        first_order_filter_cali(&para->slow_set_vx,((float)RC->rc.ch[0]/660.0f)*Chassis_Speed_MAX);
        first_order_filter_cali(&para->slow_set_vy,((float)RC->rc.ch[1]/660.0f)*Chassis_Speed_MAX);

        float after_analysis_x, after_analysis_y;
        after_analysis_y=para->slow_set_vy.out*arm_cos_f32(para->relative_yaw)-para->slow_set_vx.out*arm_sin_f32(para->relative_yaw);//坐标变换
        after_analysis_x=para->slow_set_vy.out*arm_sin_f32(para->relative_yaw)+para->slow_set_vx.out*arm_cos_f32(para->relative_yaw);

        para->speed_set.y=after_analysis_y;
        para->speed_set.x=after_analysis_x;
        para->speed_set.z=PID_calc(&follow_gimbal_v,Gimbal.Mother.yaw_motor_ecd,0);//para->slow_set_vz.out;

        para->Rotate_speed=0;

	} 
	else if(*mode==Auto)
	{


//        float temp_x=((float)RC->rc.ch[0]/660.0f)*Chassis_Speed_MAX;
//        float temp_y=((float)RC->rc.ch[1]/660.0f)*Chassis_Speed_MAX;
//        first_order_filter_cali(&para->slow_set_vx,temp_x);
//        first_order_filter_cali(&para->slow_set_vy,temp_y);

        if(robot_state.power_management_chassis_output==1)
        {
            first_order_filter_cali(&para->slow_set_vx,Navigate.speed_x);
            first_order_filter_cali(&para->slow_set_vy,Navigate.speed_y);
        }
        else
        {
            first_order_filter_cali(&para->slow_set_vx,0);
            first_order_filter_cali(&para->slow_set_vy,0);
        }




        float after_analysis_x = 0.0f, after_analysis_y = 0.0f;
        after_analysis_y=para->slow_set_vy.out*arm_cos_f32(para->relative_yaw)-para->slow_set_vx.out*arm_sin_f32(para->relative_yaw);//坐标变换
        after_analysis_x=para->slow_set_vy.out*arm_sin_f32(para->relative_yaw)+para->slow_set_vx.out*arm_cos_f32(para->relative_yaw);

#if Chassis_test_mode==0  //小陀螺
        float pow_sum=para->slow_set_vx.out * para->slow_set_vx.out + para->slow_set_vy.out * para->slow_set_vy.out;
        para->Rotate_speed=Gyro_Speed*(1-(pow_sum/((Chassis_Speed_MAX * Chassis_Speed_MAX)*2)));
#endif

		para->speed_set.y=after_analysis_y;
		para->speed_set.x=after_analysis_x;
        para->speed_set.z=para->Rotate_speed;
        constrain(&para->speed_set.y,Chassis_Speed_MAX,-Chassis_Speed_MAX);
        constrain(&para->speed_set.x,Chassis_Speed_MAX,-Chassis_Speed_MAX);

	}
	else
	{
		para->speed_set.x=0;
		para->speed_set.y=0;
		para->speed_set.z=0;
        para->Rotate_speed=0;
	}

}

void Chassis_kinematics_calc(Chassis_para *para)
{
	para->motor[2].vel_set=(para->speed_set.y*SQRT2_HALF+para->speed_set.x*SQRT2_HALF+para->speed_set.z*CHASSIS_RADIUS)/WHEEL_RADIUS;
	para->motor[3].vel_set=(para->speed_set.y*SQRT2_HALF-para->speed_set.x*SQRT2_HALF+para->speed_set.z*CHASSIS_RADIUS)/WHEEL_RADIUS;
	para->motor[0].vel_set=(-para->speed_set.y*SQRT2_HALF-para->speed_set.x*SQRT2_HALF+para->speed_set.z*CHASSIS_RADIUS)/WHEEL_RADIUS;
	para->motor[1].vel_set=(para->speed_set.x*SQRT2_HALF-para->speed_set.y*SQRT2_HALF+para->speed_set.z*CHASSIS_RADIUS)/WHEEL_RADIUS;
}

void power_control(power_model_t* para)
{
    if (/*cap.state==0 &&*/ cap.V_cap>=11.f)
    {
        power_model.power_MAX=200.f;
    }
    else
    {
        power_model.power_MAX=90.f;
    }

    for (int i = 0; i < 4; ++i)
    {
        para->power_predict[i]=Chassis.motor[i].vel*Chassis.motor[i].torque_set
                                  +P_K1*Chassis.motor[i].vel*Chassis.motor[i].vel
                                  +P_K2*Chassis.motor[i].torque_set*Chassis.motor[i].torque_set
                                  +P_a;
    }

    para->power_predict_all=para->power_predict[0]+para->power_predict[1]
            +para->power_predict[2]+para->power_predict[3];      //计算出底盘总功率

    if(para->power_predict_all>para->power_MAX)//若底盘总功率大于设定功率，则进入功率分配
    {
        para->K_scale=para->power_MAX/para->power_predict_all;//计算分配系数K
        for (int i = 0; i < 4; ++i)//依次对四个电机单独分配功率
        {
            if(para->power_predict[i]>0)
            {

                para->power_limit[i]=para->power_predict[i]*para->K_scale;//根据分配系数进行限幅
                float b=Chassis.motor[i].vel;
                float c=P_K1*Chassis.motor[i].vel*Chassis.motor[i].vel+P_a-para->power_limit[i];//计算出公式中的b,c,a

                float temp=b*b-4*P_K2*c;//解算方程的判别式
                if(temp<0.f) temp=0.00001f;//防止硬件浮点死区导致的nan错误

                if(Chassis.motor[i].torque_set>=0)
                    para->Torque_limit[i]=(-b+ sqrtf(temp))/(2*P_K2);
                else
                    para->Torque_limit[i]=(-b- sqrtf(temp))/(2*P_K2);//功率分配公式的最终体现，单电机的功率控制完成
            }
            else
            {para->Torque_limit[i]=Chassis.motor[i].torque_set;}
        }
    }
    else                                                    //若底盘总功率小于设定功率，则不进入功率分配
    {
        for (int i = 0; i < 4; ++i) {
            para->Torque_limit[i]=Chassis.motor[i].torque_set;
        }
    }
}



/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
            first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

