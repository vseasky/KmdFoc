#ifndef _BSP_SVPWM_H_
#define _BSP_SVPWM_H_

#include "main.h"


extern uint16_t adc_value[1];

/**
 * @description: svpwm初始化函数
 * @return {*}
 */
void bsp_svpwm_init(void);

/**
 * @description: SVPWM赋值
 * @param {uint16_t} duty_a
 * @param {uint16_t} duty_b
 * @param {uint16_t} duty_c
 * @return {*}
 */
void bsp_svpwm_apply_duty(uint16_t duty_a,uint16_t duty_b,uint16_t duty_c);

/**
 * @description: 开启SVPWM
 * @return {*}
 */
void bsp_svpwm_switch_on(void);


/**
 * @description: 关闭SVPWM
 * @return {*}
 */
void bsp_svpwm_switch_off(void);

/**
 * @description: foc计算中断回调，取adc数据
 * @return {*}
 */
void bsp_svpwm_irq_callback(void);
#endif
