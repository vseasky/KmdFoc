#ifndef _HAL_FOC_HAL_FSM_
#define _HAL_FOC_HAL_FSM_

#include <hal_foc_struct.h>

// Errors
#define ERR_OVER_VOLTAGE    ((uint16_t)1 << MOTOR_ERR_OVER_VOLTAGE)
#define ERR_UNDER_VOLTAGE   ((uint16_t)1 << MOTOR_ERR_UNDER_VOLTAGE)
#define ERR_OVER_SPEED      ((uint16_t)1 << MOTOR_ERR_OVER_SPEED)
#define ERR_OVER_CURRENT    ((uint16_t)1 << MOTOR_ERR_OVER_CURRENT)
#define ERR_ENCODER         ((uint16_t)1 << MOTOR_ERR_ENCODER)
#define ERR_CAN_CONFLICT    ((uint16_t)1 << MOTOR_ERR_CAN_CONFLICT)

#define ERR_HANLDER_IRQ     ((uint16_t)1 << MOTOR_ERR_HANLDER)     

// 有限状态机切换
int hal_fsm_input(tHalFocStruct *pHalFoc, tHalFocFsmCmd fsmCmd);

// 有限状态机管理
void hal_fsm_loop(tHalFocStruct *pHalFoc);

// 状态保护检测
void hal_fsm_protect(tHalFocStruct *pHalFoc);

// 获取运行模式
static inline tHalFsmStat hal_get_fsm_mode_(tHalFocFsm *pFocFsm)
{
    return pFocFsm->state;
}

// 获取错误状态
static inline uint32_t hal_get_fsm_error(tHalFocFsm *pFocFsm)
{
    return pFocFsm->motor_err;
}

// 获取运行状态
static inline uint32_t hal_get_fsm_state(tHalFocFsm *pFocFsm)
{
    /*
        |tHalFsmStat	|	Error	|
        |16bit   	    |	16bit	|
    */
    return (((uint16_t)pFocFsm->state) << 16) | (pFocFsm->motor_err & 0XFFFF);
}

// CAN ID冲突
static inline void hal_fsm_can_err(tHalFocFsm *pFocFsm)
{
    pFocFsm->motor_err |= ERR_CAN_CONFLICT;
}

// 解析状态
static inline void hal_parse_fsm_state(uint16_t *pFsm, uint16_t *pErr, uint32_t tData)
{
    *pFsm = tData >> 16;
    *pErr = tData & 0XFFFF;
}

#endif
