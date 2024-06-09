#include "bsp_interface.h"

#include "drv_usb_hw.h"
#include "cdc_acm_core.h"
#include "drv_usbd_int.h"

/* bsp 层添加hal层交互接口 */
#include "hal_foc_interact.h"
#include "hal_foc_current.h"
#include "bsp_gpio.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_kmd.h"


usb_core_driver cdc_acm;

extern tHalFocStruct mHalFocStruct;

extern bool hal_receive_callback(tHalFocStruct *pHalFoc, hal_frame_struct *rx_frame);

/**
 * @description: 初始化硬件外设
 * @return {*}
 */
void bsp_board_init(void)
{
#ifdef KMD_USB_USER
    // 初始化USB
    usb_rcu_config();
    usb_timer_init();
    usbd_init(&cdc_acm, USB_CORE_ENUM_FS, &cdc_desc, &cdc_class);
    usb_intr_config();
#ifdef USE_IRC48M
    // CTC peripheral clock enable
    rcu_periph_clock_enable(RCU_CTC);
    // CTC configure 
    ctc_config();
    while (ctc_flag_get(CTC_FLAG_CKOK) == RESET)
        {
        }
#endif
#endif
    // 初始化LED
    bsp_led_init();
    // 初始化CAN0
    bsp_can0_init();
    // 初始化串口
    bsp_uart_init();
    // 初始化DMA
    bsp_dma_init();
}


/**
 * @description: 提供给hal层的消息入口
 * @param {bsp_frame} *rx_frame
 * @return {*}
 */
bool bsp_receive_callback(bsp_frame *rx_frame)
{
    /*加入*/
    return hal_receive_callback(&mHalFocStruct,rx_frame);
}

/**
 * @description: CAN方式发送消息
 * @param {uint32_t} frameID
 * @param {uint8_t} *pData
 * @param {uint8_t} len
 * @return {*}
 */
bool bsp_can__transmit_callback(uint32_t frameID,uint8_t *pData,uint8_t len)
{
    bsp_frame txBspFrame;
    txBspFrame.can_id = frameID;
    memcpy(&txBspFrame.data[0],&pData[0],8);
    txBspFrame.data_len = len;
    return bsp_can0_transmit(&txBspFrame);
}

/**
 * @description: 串口方式发送
 * @param {uint32_t} frameID
 * @param {uint8_t} *pData
 * @param {uint8_t} len
 * @return {*}
 */
bool bsp_uart_transmit_callback(uint32_t frameID,uint8_t *pData,uint8_t len)
{
    bsp_frame txBspFrame;
    txBspFrame.can_id = frameID;
    memcpy(&txBspFrame.data[0],&pData[0],8);
    txBspFrame.data_len = len;
    return bsp_kmd_transmit(&txBspFrame,BSP_MSG_FRAME_UART);
}

/**
 * @description: USB方式发送
 * @param {uint32_t} frameID
 * @param {uint8_t} *pData
 * @param {uint8_t} len
 * @return {*}
 */
bool bsp_usb__transmit_callback(uint32_t frameID,uint8_t *pData,uint8_t len)
{
    bsp_frame txBspFrame;
    txBspFrame.can_id = frameID;
    memcpy(&txBspFrame.data[0],&pData[0],8);
    txBspFrame.data_len = len;
    return bsp_kmd_transmit(&txBspFrame,BSP_MSG_FRAME_USB);
}

/**
 * @description: 波形打印函数接口
 * @param {uint16_t} frameID
 * @param {float} *pData
 * @param {uint16_t} len
 * @return {*}
 */
bool bsp_uart_debug_port_callback(uint16_t frameID,float *pData, uint16_t len)
{
    return bsp_kmd_uart_debug(frameID,pData,len);
}

/**
 * @description: 系统复位函数
 * @return {*}
 */
void bsp_system_reset(void)
{
    __set_PRIMASK(1);
    NVIC_SystemReset();
}

/**
 * @description: svpwm中断\foc计算回调函数
 * @param {uint16_t} adc_vbus
 * @param {uint16_t *} adc_cur_value
 * @return    
 */
void bsp_svpwm_callback(uint16_t adc_vbus,uint16_t * adc_cur_value)
{
    hal_foc_calc_callback(&mHalFocStruct,adc_vbus,adc_cur_value);
}
