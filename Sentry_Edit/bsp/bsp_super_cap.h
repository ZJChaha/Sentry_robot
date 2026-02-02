//
// Created by ZJC on 2025/2/8.
//

#ifndef BSP_SUPER_CAP_H
#define BSP_SUPER_CAP_H
#include "stdint.h"


//控制器模式枚举
enum CAP_MODE
{
    CAP_MODE_NONAUTOMATIC = 1, //人控
    CAP_MODE_DISENABLED, //失能
    CAP_MODE_AUTOMATIC, //自动
    CAP_MODE_RESTAER, //重启或自检
};
//电容组&功控板控制结构体
typedef struct
{
    uint16_t P_in; //限制功率
    uint16_t P_recharge; //电容组充电功率上限
    uint16_t P_compensation; //电容组放电功率上限
    enum CAP_MODE cap_mode; //电容模式控制位
    uint8_t buffer; //剩余缓冲能量
} cap_control;
//电容组&功控板数据接收结构体
typedef struct
{
    float V_cap; //电容组电压
    float P_cap; //电容组功率
    float P_in; //电源管理输出功率
    float P_chassis; //底盘消耗功率
    enum CAP_MODE cap_mode; //运行状态
    unsigned char state; //自检状态
} cap_receive;
//结构体
extern cap_control control;
extern cap_receive cap;
//电容组&功控板数据解析
void Cap_Data_Receive(uint8_t *data,cap_receive*p);











#endif //BSP_SUPER_CAP_H
