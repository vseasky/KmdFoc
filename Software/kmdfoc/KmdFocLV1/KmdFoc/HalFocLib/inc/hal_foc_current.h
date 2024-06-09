#ifndef _HAL_FOC_CALC_H_
#define _HAL_FOC_CALC_H_

#include <hal_foc_struct.h>

static inline void hal_foc_update_current_gain(tHalUsrConfig *pUsrConfig)
{
	pUsrConfig->current_ctrl_p_gain = pUsrConfig->motor_phase_inductance * pUsrConfig->current_ctrl_bandwidth;
	pUsrConfig->current_ctrl_i_gain = pUsrConfig->motor_phase_resistance * pUsrConfig->current_ctrl_bandwidth;
}


// FOC计算回调函数，在中断接口中提供正确的编码器、总线电压、三相电流采样值数据
void hal_foc_calc_callback(tHalFocStruct *const pHalFoc, uint16_t adc_vbus, uint16_t *adc_cur_value);

// 0电流校准启动函数
void hal_foc_zero_current_start(tHalFocCurrentStruct *pFocCurrent);

// 使能输出
void hal_foc_arm(const tHalFocInterface *const pCallbcakFun);

// 失能输出
void hal_foc_disarm(const tHalFocInterface *const pCallbcakFun);

// FOC闭环电流相关参数清零
void hal_foc_current_reset(tHalFocCurrentStruct *pFocCurrent);

// 施加开环电压，提供给校准使用
void hal_apply_voltage_timings(tHalFocCurrentStruct *pFocCurrent, float vbus, float v_d, float v_q, float pwm_phase);

// FOC闭环,电流输出控制，FOC变换 Clarke变换->>Park变换->>Inverse park变换->>Apply duty
void hal_foc_current(tHalFocCurrentStruct *pFocCurrent);

#endif

