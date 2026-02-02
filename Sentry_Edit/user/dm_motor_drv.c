#include <string.h>
#include "dm_motor_drv.h"
#include "can.h"
#include "bsp_dwt.h"
#include "Chassis_Task.h"
#include "cmsis_os.h"
#include "CAN_receive.h"

dm_motor_t rub_wheel[4];


void dm_motor_init(void)
{


    memset(&rub_wheel[Motor1], 0, sizeof(rub_wheel[Motor1]));
    memset(&rub_wheel[Motor2], 0, sizeof(rub_wheel[Motor2]));
    memset(&rub_wheel[Motor3], 0, sizeof(rub_wheel[Motor3]));
    memset(&rub_wheel[Motor4], 0, sizeof(rub_wheel[Motor4]));


    rub_wheel[Motor1].id = 0x21;
    rub_wheel[Motor1].mst_id = 0x31;
    rub_wheel[Motor1].tmp.read_flag = 1;
    rub_wheel[Motor1].ctrl.mode 	= spd_mode;
    rub_wheel[Motor1].ctrl.vel_set 	= 0;

    rub_wheel[Motor2].id = 0x22;
    rub_wheel[Motor2].mst_id = 0x32;
    rub_wheel[Motor2].tmp.read_flag = 1;
    rub_wheel[Motor2].ctrl.mode 	= spd_mode;
    rub_wheel[Motor2].ctrl.vel_set 	= 0;

    rub_wheel[Motor3].id = 0x23;
    rub_wheel[Motor3].mst_id = 0x33;
    rub_wheel[Motor3].tmp.read_flag = 1;
    rub_wheel[Motor3].ctrl.mode 	= spd_mode;
    rub_wheel[Motor3].ctrl.vel_set 	= 0;

    rub_wheel[Motor4].id = 0x24;
    rub_wheel[Motor4].mst_id = 0x34;
    rub_wheel[Motor4].tmp.read_flag = 1;
    rub_wheel[Motor4].ctrl.mode 	= spd_mode;
    rub_wheel[Motor4].ctrl.vel_set 	= 0;



}


/**
************************************************************************
* @brief:      	dm_motor_clear_para
* @param[in]:   hcan:    fdcan struct
* @param[in]:   motor:   motor typedef struct
* @details:    	clear paraments
************************************************************************
**/
void dm_motor_clear_para(dm_motor_t *motor)
{
	motor->ctrl.kd_set 	= 0;
	motor->ctrl.kp_set	= 0;
	motor->ctrl.pos_set = 0;
	motor->ctrl.vel_set = 0;
	motor->ctrl.tor_set = 0;
	motor->ctrl.cur_set = 0;
}

/**
************************************************************************
* @brief:      	dm_motor_fbdata
* @param[in]:   motor:   motor typedef struct
* @param[in]:   rx_data:  receive data buffer
* @retval:     	void
* @details:    	analyze motor parameters
************************************************************************
**/
void dm_motor_fbdata(dm_motor_t *motor, const uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, -motor->tmp.PMAX, motor->tmp.PMAX, 16); // (-3.141593,3.141593)
	motor->para.vel = uint_to_float(motor->para.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12); // (-40.0,40.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
}

/**
************************************************************************
* @brief:      	float_to_uint: Function to convert a float to an unsigned integer
* @param[in]:   x_float:	Float value to be converted
* @param[in]:   x_min:		Minimum range value
* @param[in]:   x_max:		Maximum range value
* @param[in]:   bits: 		Bit width of the target unsigned integer
* @retval:     	Unsigned integer result
* @details:    	Maps the given float x linearly within the specified range [x_min, x_max] to an unsigned integer of the specified bit width.
************************************************************************
**/

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: Function to convert an unsigned integer to a float
* @param[in]:   x_int: Unsigned integer to be converted
* @param[in]:   x_min: Minimum range value
* @param[in]:   x_max: Maximum range value
* @param[in]:   bits:  Bit width of the unsigned integer
* @retval:     	Float result
* @details:    	Maps the given unsigned integer x_int linearly within the specified range [x_min, x_max] to a float.
************************************************************************
**/

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
************************************************************************
* @brief:      	enable_motor_mode: Function to enable motor mode
* @param[in]:   hcan:     Pointer to the CAN_HandleTypeDef structure
* @param[in]:   motor_id: Motor ID specifying the target motor
* @param[in]:   mode_id:  Mode ID specifying the mode to enable
* @retval:     	void
* @details:     Sends a command via the CAN bus to enable a specific mode on the targeted motor
************************************************************************
**/
void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;


    CAN_cmd_send(id,data,8);

}

void enable_all_motor_os(void)
{
    for(uint8_t i=1;i<=10;i++)
    {

        clear_err(&hcan1,0x21,SPD_MODE);
        osDelay(1);
        clear_err(&hcan1,0x22,SPD_MODE);
        osDelay(1);
        clear_err(&hcan1,0x23,SPD_MODE);
        osDelay(1);
        clear_err(&hcan1,0x24,SPD_MODE);
        osDelay(1);
    }

    for(uint8_t i=1;i<=20;i++)
    {
        enable_motor_mode(&hcan1,0x21,SPD_MODE);
        osDelay(1);
        enable_motor_mode(&hcan1,0x22,SPD_MODE);
        osDelay(1);
        enable_motor_mode(&hcan1,0x23,SPD_MODE);
        osDelay(1);
        enable_motor_mode(&hcan1,0x24,SPD_MODE);
        osDelay(1);
    }

}



/**
************************************************************************
* @brief:      	disable_motor_mode: Function to disable motor mode
* @param[in]:   hcan:     Pointer to the CAN_HandleTypeDef structure
* @param[in]:   motor_id: Motor ID specifying the target motor
* @param[in]:   mode_id:  Mode ID specifying the mode to disable
* @retval:     	void
* @details:     Sends a command via the CAN bus to disable a specific mode on the targeted motor
************************************************************************
**/
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;

    CAN_cmd_send( id, data, 8);
}


/**
************************************************************************
* @brief:      	clear_err: Function to clear motor error
* @param[in]:   hcan:     Pointer to the CAN_HandleTypeDef structure
* @param[in]:   motor_id: Motor ID specifying the target motor
* @param[in]:   mode_id:  Mode ID specifying the mode to clear errors
* @retval:     	void
* @details:     Sends a command via the CAN bus to clear errors on a specific motor
************************************************************************
**/
void clear_err(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;

    CAN_cmd_send(id, data, 8);
}



void spd_ctrl(hcan_t* hcan, uint16_t motor_id, float vel)
{
    uint16_t id;
    uint8_t *vbuf;
    uint8_t data[4];

    id = motor_id + SPD_MODE;
    vbuf=(uint8_t*)&vel;

    data[0] = *vbuf;
    data[1] = *(vbuf+1);
    data[2] = *(vbuf+2);
    data[3] = *(vbuf+3);

    CAN_cmd_send(id, data, 4);
}




