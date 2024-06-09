#ifndef  __BSP_UART_H__
#define  __BSP_UART_H__

#include "bsp_interface.h"

/**
 * @description: ³õÊ¼»¯´®¿Ú
 * @return {*}
 */
void bsp_uart_init(void);

/**
 * @description: ???????dma
 * @return {*}
 */
void bsp_dma_init(void);

/**
 * @description: uart ??????
 * @param {uint8_t} *pData
 * @param {uint16_t} len
 * @return {*}
 */
bool bsp_uart_transmit(uint8_t *pData,uint16_t len);

/**
 * @description: uart ???? 
 * @return {*}
 */
void bsp_uart_irq_callback(void);

#endif
