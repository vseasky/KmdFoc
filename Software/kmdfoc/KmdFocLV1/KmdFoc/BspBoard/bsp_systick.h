#ifndef _BSP_SYSTICK_H__
#define _BSP_SYSTICK_H__

#include "main.h"

extern volatile uint64_t SysTickCnt;

/**
 * @description: 初始化Systick
 * @return {*}
 */
void bsp_systick_init(void);

/**
 * @description: 获取系统时间us
 * @return {*}
 */
uint64_t bsp_systick_get_tick_us(void);

/**
 * @description: 获取系统时间差us
 * @param {uint64_t} tick_us
 * @return {*}
 */
uint64_t bsp_systick_get_us_since(uint64_t tick_us);

/**
 * @description: 获取系统时间ms
 * @return {*}
 */
uint64_t bsp_systick_get_tick_ms(void);


/**
 * @description: 计算系统时间差ms
 * @param {uint32_t} tick
 * @return {*}
 */
uint64_t bsp_systick_get_ms_since(uint64_t tick_ms);

/**
 * @description: systick us延时函数
 * @param {uint32_t} uS
 * @return {*}
 */
void bsp_systick_delay_us(uint64_t us);


/**
 * @description: ms延时函数
 * @param {uint32_t} mS
 * @return {*}
 */
void bsp_systick_delay_ms(uint64_t ms);

#endif
