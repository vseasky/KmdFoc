#ifndef _HAL_KMD_FOC_
#define _HAL_KMD_FOC_

#include <stdio.h>
#include <stdint.h>

#include "hal_foc_struct.h"

typedef struct
{
    bool (*const hal_kmd_disable_motor_hook)(hal_frame_struct *pFrame, uint16_t node_id);
    bool (*const hal_kmd_set_pisotion_hook)(hal_frame_struct *pFrame, uint16_t node_id, float tData);
    bool (*const hal_kmd_set_velocity_hook)(hal_frame_struct *pFrame, uint16_t node_id, float tData);
    bool (*const hal_kmd_set_current_hook)(hal_frame_struct *pFrame, uint16_t node_id, float tData);
    bool (*const hal_kmd_get_position_hook)(hal_frame_struct *pFrame, uint16_t node_id);
    bool (*const hal_kmd_get_velocity_hook)(hal_frame_struct *pFrame, uint16_t node_id);
    bool (*const hal_kmd_get_current_hook)(hal_frame_struct *pFrame, uint16_t node_id);
    bool (*const hal_kmd_get_bus_hook)(hal_frame_struct *pFrame, uint16_t node_id);
    bool (*const hal_kmd_get_version_hook)(hal_frame_struct *pFrame, uint16_t node_id);
    struct
    {
        bool (*const hal_kmd_fsm_motor_disable_hook)(hal_frame_struct *pFrame, uint16_t node_id);
        bool (*const hal_kmd_fsm_motor_enable_hook)(hal_frame_struct *pFrame, uint16_t node_id);
        bool (*const hal_kmd_fsm_motor_reset_error_hook)(hal_frame_struct *pFrame, uint16_t node_id);
        bool (*const hal_kmd_fsm_motor_get_state_hook)(hal_frame_struct *pFrame, uint16_t node_id);
        bool (*const hal_kmd_fsm_motor_cali_start_hook)(hal_frame_struct *pFrame, uint16_t node_id);
        bool (*const hal_kmd_fsm_motor_cali_stop_hook)(hal_frame_struct *pFrame, uint16_t node_id);
        bool (*const hal_kmd_fsm_motor_configs_updata_hook)(hal_frame_struct *pFrame, uint16_t node_id);
        bool (*const hal_kmd_fsm_motor_configs_reset_hook)(hal_frame_struct *pFrame, uint16_t node_id)
    } kmd_fsm;
    struct
    {
        bool  (*const hal_kmd_user_config_set_hook)(hal_frame_struct *pFrame, uint16_t node_id, uint16_t tOrderCmd, uint8_t *pData);
        bool  (*const hal_kmd_user_config_get_hook)(hal_frame_struct *pFrame, uint16_t node_id, uint16_t tOrderCmd);
    } kmd_config;
} hal_kmd_user_struct;

bool hal_kmd_frame_receive_callback(hal_frame_struct *pFrame, tHalFocInfo *pKmdInfo);

#endif