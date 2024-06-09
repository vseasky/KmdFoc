#ifndef _BSP_ENCODER_H_
#define _BSP_ENCODER_H_

#include <main.h>
#include "bsp_hall.h"
#include "bsp_ma730.h"
#include "bsp_mt6825.h"
#include "bsp_as5047.h"


/**
 * @description: 初始化编码器
 * @param {int} encoder_type
 * 				->>编码器类型
 * @return {*}
 */
void bsp_encoder_init(int encoder_type);

/**
 * @description: 读取编码器的值
 * @param {int} encoder_type
 * @param {uint32_t} *pReadRaw
 * 				->>编码器类型
 * @return {*}
 */
bool bsp_encoder_read_raw(int encoder_type,uint32_t *pReadRaw);


#endif
