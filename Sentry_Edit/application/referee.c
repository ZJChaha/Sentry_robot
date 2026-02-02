#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
ext_referee_warning_t referee_warning_t;
hurt_data_t hurt_data;

ext_game_robot_state_t robot_state;
ext_power_heat_data_t power_heat_data_t;
projectile_allowance_t projectile_allowance;
rfid_status_t rfid_status;

ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_bullet_remaining_t bullet_remaining_t;
ext_student_interactive_data_t student_interactive_data_t;

float init_speed_1,init_speed_2;
uint8_t fre_1,fre_2;
uint8_t hurt_receive_flag,hurt_flag;


void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));


    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));


    memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&projectile_allowance, 0, sizeof(projectile_allowance));
    memset(&rfid_status, 0, sizeof(rfid_status));
    memset(&hurt_data, 0, sizeof(hurt_data));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));


    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));



}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP_t,frame + index, sizeof(game_robot_HP_t));
        }
        break;
        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(robot_state));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
            get_init_speed();
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance));

        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&hurt_data, frame + index, sizeof(hurt_data));
            hurt_receive_flag=1;
        }
        break;
        case RFID_STATUS_ID:
        {
            memcpy(&rfid_status, frame + index, sizeof(rfid_status));
        }
        break;
        default:
        {
            break;
        }
    }
}

void get_chassis_power_and_buffer(float *power, float *buffer)
{

    *buffer = power_heat_data_t.buffer_energy;

}


uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}

void get_init_speed(void)
{
    if(shoot_data_t.shooter_number==1)
    {
        init_speed_1=shoot_data_t.initial_speed;
        fre_1=shoot_data_t.launching_frequency;
    }
    if(shoot_data_t.shooter_number==2)
    {
        init_speed_2=shoot_data_t.initial_speed;
        fre_2=shoot_data_t.launching_frequency;
    }

}
