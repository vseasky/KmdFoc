#include "hal_foc_interface.h"
#include "hal_foc_interact.h"
#include "hal_foc_struct.h"
#include "hal_foc_encoder.h"
#include "hal_foc_usr_config.h"
#include "hal_foc_current.h"
#include "hal_foc_fsm.h"

#include "bsp_interface.h"
#include "bsp_systick.h"
#include "bsp_flash.h"
#include "bsp_svpwm.h"
#include "bsp_kmd.h"
#include "bsp_gpio.h"
#include "bsp_encoder.h"
#include "bsp_device_id.h"
#define HAL_MIN_CAN__TRANSMI_MS (1)
#define HAL_MIN_UART_TRANSMI_MS (2)
#define HAL_MIN_USB__TRANSMI_MS (1)


// FLASH->根据芯片设置（存储区开始位置应至少大于 程序大小，因此尽量从可用存储区尾部选取存储位置）
#define FMC_PAGE_SIZE ((uint16_t)0x400U)                                  // 1KB  1024
#define USR_CONFIG_ROM_ADDR ((uint32_t)(0x8000000 + 100 * FMC_PAGE_SIZE)) // Page 100

void hal_foc_init(tHalFocStruct *const pHalFoc);
void hal_encoder_init(tHalEncoderStruct *const pEncoder);
bool hal_debug_port(tHalTransmitCallback *pHalTransmitCallback, uint16_t frameID, float *pData, uint16_t len);
bool hal_transmit(tHalTransmitCallback *pHalTransmitCallback, hal_frame_struct *phal_frame_struct);
bool hal_rx_transmit(tHalTransmitCallback *pHalTransmitCallback, hal_frame_struct *phal_frame_struct);

tHalUsrConfig mHalUsrConfig =
    {0};
/**
 * 初始化FOC所需函数，务必提供所有所需的底层实现函数，编译器可能无法检测出函数指针指向的函数的参数类型与其自身传递的参数类型是否一致，这种问题出现
 * 往往较难排查，因此在写底层驱动时建议直接复制函数指针然后改名作为函数名称，以保证FOC可以正常的运行，在此使用C语言抽象类实现FOC算法会极大的缩短开
 * 发周期,
 * const 函数指针
 */
tHalFocInterface mHalFocCallbackFun =
    {
        // 系统复位
        .hal_system_reset = &bsp_system_reset,
        // Systick
        .hal_systick_get_us_tick_ = *bsp_systick_get_tick_us,
        .hal_systick_get_us_since = &bsp_systick_get_us_since,
        .hal_systick_get_ms_tick_ = &bsp_systick_get_tick_ms,
        .hal_systick_get_ms_since = &bsp_systick_get_ms_since,
        .hal_systick_delay_ms = &bsp_systick_delay_ms,
        .hal_systick_delay_us = &bsp_systick_delay_us,
        // Flash
        .hal_flash_write = &bsp_flash_write,
        .hal_flash_read_ = &bsp_flash_read_,
        // SVPWM
        .hal_svpwm_init = &bsp_svpwm_init,
        .hal_svpwm_apply_duty = &bsp_svpwm_apply_duty,
        .hal_svpwm_switch_off = &bsp_svpwm_switch_off,
        .hal_svpwm_switch_on_ = &bsp_svpwm_switch_on,

        .hal_board_init = &bsp_board_init,
        // LED状态控制
        .hal_led_set = &bsp_led_state_set,
        // Encoder
        .hal_encoder_read_raw = &bsp_encoder_read_raw,
        .hal_get_device_id = &bsp_device_id_get,

        .hal_transmit_type.type_transmit = KMD_MSG_FRAME_NULL,
        .hal_transmit_type.type_debug = KMD_MSG_FRAME_UART,
        //消息发送
        .hal_transmit_type.hal_can_.hal_transmit__ = &bsp_can__transmit_callback,
        .hal_transmit_type.hal_can_.hal_debug_port = &bsp_uart_debug_port_callback,
        .hal_transmit_type.hal_can_.mTransmitTick_ms = 0,
        .hal_transmit_type.hal_can_.mTransmitCycle_ms = HAL_MIN_CAN__TRANSMI_MS,
        .hal_transmit_type.hal_can_.hal_systick_get_ms_tick_ = &bsp_systick_get_tick_ms,
        .hal_transmit_type.hal_can_.hal_systick_get_ms_since = &bsp_systick_get_ms_since,

        .hal_transmit_type.hal_uart.hal_transmit__ = &bsp_uart_transmit_callback,
        .hal_transmit_type.hal_uart.hal_debug_port = &bsp_uart_debug_port_callback,
        .hal_transmit_type.hal_uart.mTransmitTick_ms = 0,
        .hal_transmit_type.hal_uart.mTransmitCycle_ms = HAL_MIN_UART_TRANSMI_MS,
        .hal_transmit_type.hal_uart.hal_systick_get_ms_tick_ = &bsp_systick_get_tick_ms,
        .hal_transmit_type.hal_uart.hal_systick_get_ms_since = &bsp_systick_get_ms_since,

        .hal_transmit_type.hal_usb_.hal_transmit__ = &bsp_usb__transmit_callback,
        .hal_transmit_type.hal_usb_.hal_debug_port = &bsp_uart_debug_port_callback,
        .hal_transmit_type.hal_usb_.mTransmitTick_ms = 0,
        .hal_transmit_type.hal_usb_.mTransmitCycle_ms = HAL_MIN_USB__TRANSMI_MS,
        .hal_transmit_type.hal_usb_.hal_systick_get_ms_tick_ = &bsp_systick_get_tick_ms,
        .hal_transmit_type.hal_usb_.hal_systick_get_ms_since = &bsp_systick_get_ms_since,
        // Port波形打印
        .hal_port_fun = &hal_debug_port,
        // 协议数据发送
        .hal_transmit_fun = &hal_transmit,
        .hal_rx_transmit_fun = &hal_rx_transmit,
};

/**
 * @description: 设置初始化参数
 * @return {*}
 */
tHalFocStruct mHalFocStruct =
    {
        .const_define.mFocDt = HAL_DT,
        .const_define.version = {HAL_FW_VERSION_1, HAL_FW_VERSION_2, HAL_FW_VERSION_3, HAL_FW_VERSION_4},

        .mFocCore.mFocCurrent.foc_calib_zero_current.zero_current_count = 0,
        .mFocCore.mFocCurrent.foc_calib_zero_current.zero_current_start = false,
        .mFocCore.mFocCurrent.foc_calib_zero_current.zero_current_calib_valid = false,
        .mFocCore.mFocCurrent.foc_calib_zero_current.zero_current_max_count = HAL_ZERO_CALIB_MAX_COUNT,
        .mFocCore.mFocCurrent.foc_apply_duty = bsp_svpwm_apply_duty,

        .mFocCore.mFocCurrent.const_define.current_filt_alpha = HAL_CURRENT_FILT_ALPHA_DEF,
        .mFocCore.mFocCurrent.const_define.vbus_filt_alpha = HAL_VBUS_FILT_ALPHA_DEF,
        .mFocCore.mFocCurrent.const_define.hal_v_scale = HAL_V_SCALE_DEF,
        .mFocCore.mFocCurrent.const_define.hal_i_scale = HAL_I_SCALE_DEF,
        .mFocCore.mFocCurrent.const_define.hal_max_current = HAL_MAX_CURRENT_DEF * 0.85f,
        .mFocCore.mFocCurrent.const_define.hal_pwm_arr = HAL_PWM_ARR_DEF,
        .mFocCore.mFocCurrent.const_define.mFocDt = HAL_DT,

        .mFocCore.mFocCurrent.usr_config.current_ctrl_i_gain = &mHalUsrConfig.current_ctrl_i_gain,
        .mFocCore.mFocCurrent.usr_config.current_ctrl_p_gain = &mHalUsrConfig.current_ctrl_p_gain,
        .mFocCore.mFocCurrent.usr_config.encoder_dir = &mHalUsrConfig.encoder_dir,

        .mFocCore.mFocEncoder.const_define.mFocDt = HAL_DT,
        .mFocCore.mFocEncoder.usr_config.calib_valid = &mHalUsrConfig.calib_valid,
        .mFocCore.mFocEncoder.usr_config.encoder_cpr = &mHalUsrConfig.encoder_cpr,
        .mFocCore.mFocEncoder.usr_config.encoder_dir = &mHalUsrConfig.encoder_dir,
        .mFocCore.mFocEncoder.usr_config.encoder_offset_float = &mHalUsrConfig.encoder_offset_float,
        .mFocCore.mFocEncoder.usr_config.encoder_pll_bandwidth = &mHalUsrConfig.encoder_pll_bandwidth,
        .mFocCore.mFocEncoder.usr_config.encoder_type = &mHalUsrConfig.encoder_type,
        .mFocCore.mFocEncoder.usr_config.motor_pole_pairs = &mHalUsrConfig.motor_pole_pairs,
        .mFocCore.mFocEncoder.usr_config.offset_lut = mHalUsrConfig.offset_lut,
        .mFocCore.mFocEncoder.encoder_err_check = true,
        .mFocCore.mFocEncoder.encoder_valid = true,

        .mFocCore.mFocCalib.const_define.num_R_ki = 2.0f,              //电阻测量Ki参数
        
        
        .mFocCore.mFocCalib.const_define.mCalibResTimeS = 2.0f,        //电阻测量时间
        .mFocCore.mFocCalib.const_define.mCalibIndTimeS = 1.0f,        //电感测量时间
        .mFocCore.mFocCalib.const_define.mCalibDirTimeS = 2.0f,        //极性校准电流提升时间
        .mFocCore.mFocCalib.const_define.mCalibEncTimeS = 2.0f,        //编码器校准电流提升时间
        .mFocCore.mFocCalib.const_define.mCalibSpeed = HAL_CALB_SPEED, //编码器校准转速
        .mFocCore.mFocCalib.const_define.mReportTimeS = 0.025f,         //上报周期限制
        .mFocCore.mFocCalib.const_define.mCalibMaxPairs = HAL_MOTOR_POLE_PAIRS_MAX,
        .mFocCore.mFocCalib.const_define.mFocDt = HAL_DT, //校准周期,单位s
        .mFocCore.mFocCalib.p_error_arr = NULL,

        .mFocCore.mFocFsm.state = HAL_FSM_STARTUP,
        .mFocCore.mFocFsm.next_state = HAL_FSM_STARTUP,
        .mFocCore.mFocFsm.ready = 1,
        .mFocCore.mFocFsm.motor_err = 0,
        .mFocCore.mFocFsm.motor_err_last = 0,
        //参数相关
        .mFocUser.pUsrConfig = &mHalUsrConfig,
        //参数保存地址
        .mFocUser.mUsrConfigAddress = USR_CONFIG_ROM_ADDR,
        //回调函数
        .mFocUser.pCallbackFun = &mHalFocCallbackFun,
        //消息发送
        .mFocUser.report_heart_cnt = 0,
};

bool hal_debug_port_type(tHalTransmitCtr *pHalTransmitCtr, uint16_t frameID, float *pData, uint16_t len)
{
    //超过消息额定周期的数据不发送
    if (pHalTransmitCtr->hal_systick_get_ms_since(pHalTransmitCtr->mTransmitTick_ms) >= pHalTransmitCtr->mTransmitCycle_ms)
    {
        pHalTransmitCtr->mTransmitTick_ms = pHalTransmitCtr->hal_systick_get_ms_tick_();
        return pHalTransmitCtr->hal_debug_port(frameID, pData, len);
    }
    return false;
}

/**
 * @Description  : 普通方式发送数据，允许丢失部分数据
 * @param         (tHalTransmitCtr) *pHalTransmitCtr
 * @return        (*)
 */
bool hal_transmit_type(tHalTransmitCtr *pHalTransmitCtr, hal_frame_struct *phal_frame_struct)
{
    //超过消息额定周期的数据不发送
    if (pHalTransmitCtr->hal_systick_get_ms_since(pHalTransmitCtr->mTransmitTick_ms) >= pHalTransmitCtr->mTransmitCycle_ms)
    {
        pHalTransmitCtr->mTransmitTick_ms = pHalTransmitCtr->hal_systick_get_ms_tick_();
        return pHalTransmitCtr->hal_transmit__(phal_frame_struct->can_id, phal_frame_struct->data, phal_frame_struct->data_len);
    }
    return false;
}

/**
 * @Description  : 接收请求的数据发送，不允许丢失数据
 * @param         (tHalTransmitCtr) *pHalTransmitCtr
 * @return        (*)
 */
bool hal_rx_transmit_type(tHalTransmitCtr *pHalTransmitCtr, hal_frame_struct *phal_frame_struct)
{
    bool ret = false;
    pHalTransmitCtr->mTransmitTick_ms = pHalTransmitCtr->hal_systick_get_ms_tick_();
    ret = pHalTransmitCtr->hal_transmit__(phal_frame_struct->can_id, phal_frame_struct->data, phal_frame_struct->data_len);
    return ret;
}

/**
 * @description: 初始化foc，外部接口
 * @return {*}
 */
void hal_foc_init_main(void)
{
    hal_foc_init(&mHalFocStruct);
}

/**
 * @description:foc自动上报检查
 * @return {*}
 */
void hal_foc_check_loop(void)
{
    hal_check_loop(&mHalFocStruct);
}

/**
 * @description: FOC进入异常中断，上报
 * @return {*}
 */
bool hal_report_err_hanlder(void)
{
    hal_frame_struct txFrame;
    txFrame.msg_type = mHalFocStruct.mFocUser.pCallbackFun->hal_transmit_type.type_transmit;
    txFrame.data_len = 8;
    mHalFocStruct.mFocCore.mFocFsm.motor_err |= ERR_HANLDER_IRQ;
    hal_make_frame_id(mHalFocStruct.mFocUser.pUsrConfig->can_id, HAL_KMD_MOTOR_CTR, &txFrame.can_id);
    uint32_to_data(HAL_MOTOR_ERROR_REPORT, &txFrame.data[0]);
    uint32_to_data(hal_get_fsm_state(&mHalFocStruct.mFocCore.mFocFsm), &txFrame.data[4]);
    return mHalFocStruct.mFocUser.pCallbackFun->hal_transmit_fun(&mHalFocStruct.mFocUser.pCallbackFun->hal_transmit_type, &txFrame);
}

/**
 * @description: 初始化foc
 * @param {tHalFocStruct} *pHalFoc
 * @return {*}
 */
void hal_foc_init(tHalFocStruct *const pHalFoc)
{
    // 读取Foc存储参数
    if (0 != hal_usr_read_config(&pHalFoc->mFocUser))
    {
        hal_usr_set_default_config(&pHalFoc->mFocUser);
        hal_usr_save_config(&pHalFoc->mFocUser);
    }
    pHalFoc->mFocUser.pCallbackFun->hal_systick_delay_ms(20);
    // 初始化编码器，部分编码器必须等待上电一段时间后，才能读取值
    hal_encoder_init(&pHalFoc->mFocCore.mFocEncoder);
    // 初始化输出，开启中断采样
    pHalFoc->mFocUser.pCallbackFun->hal_svpwm_init();
    hal_foc_disarm(pHalFoc->mFocUser.pCallbackFun);
    // 等待5ms,再计算零电流
    pHalFoc->mFocUser.pCallbackFun->hal_systick_delay_ms(5);
    // 等待稳定
    hal_foc_zero_current_start(&pHalFoc->mFocCore.mFocCurrent);
    // 等待校准完成
    while (pHalFoc->mFocCore.mFocCurrent.foc_calib_zero_current.zero_current_calib_valid == false)
    {
        pHalFoc->mFocUser.pCallbackFun->hal_systick_delay_ms(2);
    }
    // 初始化基础通信外设，此后允许用户进行通信控制
    pHalFoc->mFocUser.pCallbackFun->hal_board_init();
    pHalFoc->mFocUser.pCallbackFun->hal_systick_delay_ms(5);
    // 清除启动过程电流和电压波动造成的错误标志
    hal_fsm_input(&mHalFocStruct, HAL_CMD_FSM_RESET_ERROR);
    pHalFoc->mFocUser.pCallbackFun->hal_systick_delay_ms(5);
    hal_fsm_input(&mHalFocStruct, HAL_CMD_FSM_MENU);
}

void hal_encoder_init(tHalEncoderStruct *const pEncoder)
{
    bsp_encoder_init((*pEncoder->usr_config.encoder_type));
    switch ((*pEncoder->usr_config.encoder_type))
    {
    case HAL_ENCODER_MT6825:
    {
        *pEncoder->usr_config.encoder_cpr = BSP_MT6825_ENCODER_CPR;
        pEncoder->enc_cpr = BSP_MT6825_ENCODER_CPR;
        pEncoder->enc_cpr_bit = BSP_MT6825_ENCODER_CPR_BIT;
        pEncoder->enc_cpr_drv_2 = (pEncoder->enc_cpr / 2);
        pEncoder->enc_cpr_offset = (pEncoder->enc_cpr_bit - 7);
    };
    break;
    case HAL_ENCODER_MA730:
    {
        *pEncoder->usr_config.encoder_cpr = BSP_MA730_ENCODER_CPR;
        pEncoder->enc_cpr = BSP_MA730_ENCODER_CPR;
        pEncoder->enc_cpr_bit = BSP_MA730_ENCODER_CPR_BIT;
        pEncoder->enc_cpr_drv_2 = (pEncoder->enc_cpr / 2);
        pEncoder->enc_cpr_offset = (pEncoder->enc_cpr_bit - 7);
    };
    break;
    case HAL_ENCODER_AS5047P:
    {
        *pEncoder->usr_config.encoder_cpr = BSP_AS5047_ENCODER_CPR;
        pEncoder->enc_cpr = BSP_AS5047_ENCODER_CPR;
        pEncoder->enc_cpr_bit = BSP_AS5047_ENCODER_CPR_BIT;
        pEncoder->enc_cpr_drv_2 = (pEncoder->enc_cpr / 2);
        pEncoder->enc_cpr_offset = (pEncoder->enc_cpr_bit - 7);
    };
    break;
    case HAL_ENCODER_HALL:
    {
        // HALL 编码器
        pEncoder->enc_cpr = *pEncoder->usr_config.encoder_cpr;
        pEncoder->enc_cpr_bit = 0;
        pEncoder->enc_cpr_drv_2 = (pEncoder->enc_cpr / 2);
        pEncoder->enc_cpr_offset = 0;
    };
    break;
    case HAL_ENCODER_ABI:
    {
        // abi 编码器
        pEncoder->enc_cpr = *pEncoder->usr_config.encoder_cpr;
        pEncoder->enc_cpr_bit = 0;
        pEncoder->enc_cpr_drv_2 = (pEncoder->enc_cpr / 2);
        pEncoder->enc_cpr_offset = 0;
    };
    break;
    default:
    {
        *pEncoder->usr_config.encoder_cpr = BSP_MT6825_ENCODER_CPR;
        pEncoder->enc_cpr = BSP_MT6825_ENCODER_CPR;
        pEncoder->enc_cpr_bit = BSP_MT6825_ENCODER_CPR_BIT;
        pEncoder->enc_cpr_drv_2 = (pEncoder->enc_cpr / 2);
        pEncoder->enc_cpr_offset = (pEncoder->enc_cpr_bit - 7);
    };
    break;
    }
    hal_encoder_reset_def(pEncoder);
}

// 自动控制发送时间
bool hal_debug_port(tHalTransmitCallback *pHalTransmitCallback, uint16_t frameID, float *pData, uint16_t len)
{
    switch (pHalTransmitCallback->type_debug)
    {
    // 若通信方式还未进行初始化，则直接视为成功，以避免阻塞调用
    case KMD_MSG_FRAME_NULL:
    {
        return true;
    }
    break;
    case KMD_MSG_FRAME_CAN_:
    {
        return hal_debug_port_type(&pHalTransmitCallback->hal_can_, frameID, pData, len);
    }
    break;
    case KMD_MSG_FRAME_UART:
    {
        return hal_debug_port_type(&pHalTransmitCallback->hal_uart, frameID, pData, len);
    }
    break;
    case KMD_MSG_FRAME_USB_:
    {
        return hal_debug_port_type(&pHalTransmitCallback->hal_usb_, frameID, pData, len);
    }
    break;
    default:
        break;
    }
    return false;
}

/**
 * @Description  : 普通方式发送数据，允许丢失部分数据
 * @param         (tHalTransmitCallback) *pHalTransmitCallback
 * @return        (*)
 */
bool hal_transmit(tHalTransmitCallback *pHalTransmitCallback, hal_frame_struct *phal_frame_struct)
{
    bool ret = false;
    //按消息接收到的消息类型发送
    switch (phal_frame_struct->msg_type)
    {
    //若通信方式还未进行初始化，则直接视为成功，以避免阻塞调用
    case KMD_MSG_FRAME_NULL:
    {
        ret = true;
    }
    break;
    case KMD_MSG_FRAME_CAN_:
    {
        ret = hal_transmit_type(&pHalTransmitCallback->hal_can_, phal_frame_struct);
    }
    break;
    case KMD_MSG_FRAME_UART:
    {
        ret = hal_transmit_type(&pHalTransmitCallback->hal_uart, phal_frame_struct);
    }
    break;
    case KMD_MSG_FRAME_USB_:
    {
        ret = hal_transmit_type(&pHalTransmitCallback->hal_usb_, phal_frame_struct);
    }
    break;
    default:
        break;
    }
    return ret;
}

/**
 * @Description  : 请求数据需要一定的保障性,避免1ms内有进行数据上报，而带来的数据无法上报的问题
 * @param         (tHalTransmitCallback) *pHalTransmitCallback
 * @return        (*)
 */
bool hal_rx_transmit(tHalTransmitCallback *pHalTransmitCallback, hal_frame_struct *phal_frame_struct)
{
    bool ret = false;
    //按消息接收到的消息类型发送
    switch (phal_frame_struct->msg_type)
    {
    //若通信方式还未进行初始化，则直接视为成功，以避免阻塞调用
    case KMD_MSG_FRAME_NULL:
    {
        ret = true;
    }
    break;
    case KMD_MSG_FRAME_CAN_:
    {
        ret = hal_rx_transmit_type(&pHalTransmitCallback->hal_can_, phal_frame_struct);
    }
    break;
    case KMD_MSG_FRAME_UART:
    {
        ret = hal_rx_transmit_type(&pHalTransmitCallback->hal_uart, phal_frame_struct);
    }
    break;
    case KMD_MSG_FRAME_USB_:
    {
        ret = hal_rx_transmit_type(&pHalTransmitCallback->hal_usb_, phal_frame_struct);
    }
    break;
    default:
        break;
    }
    return ret;
}


//void hal_foc_irq_callback(tHalFocStruct *const pHalFoc, uint16_t adc_vbus, uint16_t *adc_cur_value)
//{
//     hal_foc_calc_callback(&mHalFocStruct,adc_vbus,adc_cur_value);
//}