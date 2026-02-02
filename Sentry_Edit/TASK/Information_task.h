#ifndef INFORMATION_TASK_H
#define INFORMATION_TASK_H

#include "stdint.h"
#include "All_struct.h"

//条件编译判断项
#define Gimbal_test_mode 1  // 1:初始化子云台回零 其他：无
#define Chassis_test_mode 0 //0:变速小陀螺  1:无小陀螺
//#define Shoot_test_mode 0 //0:瞄准后开火  1:直接开火
#define Vision_detect_mode 1 //置1开启miniPC挂机检测
//#define Game_Mode 0  //置1为比赛模式

#define NAVIGATE_TEST 0 //置1导航测试，手动子云台。置0扫描







#define Yaw_Scan_Speed 1.2f
#define Pitch_Scan_Speed 1.3f


extern uint8_t Mode;
extern uint8_t vision_detect_flag,navigation_detect_flag,game_mode;
extern uint8_t select_mode_finish,Auto_Mode_policy;

enum Mode
{

Stop,//断掉所有电机的输出	
Remote_Chassis,//遥控底盘和母云台
Auto,//自动模式
Lock,//锁定所有电机

};





extern Flag_t flag;


void Information_task(void const *pvParameters);
void Set_Mode(uint8_t* mode);
void sub_gimbal_auto_init(gimbal_para* L_para,gimbal_para* R_para);
int abs_i(int n);
void Information_update(void);
void Kalman_filter_init(void);
void Navigation_info_send(void);
void SoftReset(void);

#endif