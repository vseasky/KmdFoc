#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#include "stdbool.h"
#include "stdint.h"
#include "hal_foc_interface.h"

/**
 * @description: CAN初始化
 * @return {*}
 */
void bsp_can0_init(void);

/**
 * @description: CAN消息发送函数
 * @return {*}
 */
bool bsp_can0_transmit(hal_frame_struct *tx_frame);

/**
 * @description: can 中断回调
 * @return {*}
 */
void bsp_can0_irq_callback(void);


#endif
