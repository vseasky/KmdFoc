#ifndef _BSP_HALL_H
#define _BSP_HALL_H

#include "gd32c10x.h"
#include "stdbool.h"

/**
 * @description: 初始化HALL编码器接口
 * @return {*}
 */
void bsp_hall_init(void);


/**
 * @description: 读取HALL编码器的值
 * @param {uint32_t} *pReadRaw
 * @return {*}
 */
bool bsp_hall_read_raw(uint32_t *pReadRaw);


#endif
