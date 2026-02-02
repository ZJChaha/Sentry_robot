/**
  * @author  Liu heng
  * ¿¨¶ûÂüÂË²¨Æ÷À´×ÔRoboMasterÂÛÌ³  
  */
  
#ifndef _KALMAN_H
#define _KALMAN_H
#include "All_struct.h"


typedef struct
{
    extKalman_t Vision_pitch;
    extKalman_t Vision_yaw;
    extKalman_t Vision_distance;
}gimbal_info_t;

typedef struct
{
    gimbal_info_t Sub_L;
    gimbal_info_t Sub_R;
}Kalman_gimbal_info_t;



//´´½¨ÂË²¨Æ÷
void KalmanCreate(extKalman_t *p,float T_Q,float T_R);

//ÍùÂË²¨Æ÷ÈûÊı¾İ
float KalmanFilter(extKalman_t* p,float dat);


//Çå¿Õ¿¨¶ûÂüÂË²¨Æ÷
void KalmanClear(extKalman_t *p);

#endif
