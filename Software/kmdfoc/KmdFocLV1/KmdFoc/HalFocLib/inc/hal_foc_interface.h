#ifndef _HAL_FOC_INTERFACE_H_
#define _HAL_FOC_INTERFACE_H_

//标准头文件
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef _HAL_DEBUG__
#define HAL_DEBUG(...) printf(__VA_ARGS__);
#else
#define HAL_DEBUG(...)
#endif

#ifdef HAL_GD32_LIB_USED
#include "gd32c10x.h"
#endif

//通信方式
typedef enum eHalKmdMsgType
{
	KMD_MSG_FRAME_NULL = 0,
	KMD_MSG_FRAME_CAN_,
	KMD_MSG_FRAME_UART,
	KMD_MSG_FRAME_USB_,
} tHalKmdMsgType;

typedef struct hal_frame
{
	uint32_t can_id;		 // 数据ID
	tHalKmdMsgType msg_type; // 数据通信方式
	uint8_t can_dlc;
	uint8_t data[8];   // 数据内容
	uint16_t data_len; // 数据长度
} hal_frame_struct;

typedef struct eHalTransmitCtr
{
	uint32_t mTransmitTick_ms;
	const uint32_t mTransmitCycle_ms;
	// transmit
	bool (*const hal_transmit__)(uint32_t frameID, uint8_t *pData, uint8_t len);
	// 打印数据波形
	bool (*const hal_debug_port)(uint16_t frameID, float *pData, uint16_t len);
	// Systick
	uint32_t (*const hal_systick_get_ms_tick_)(void);
	uint32_t (*const hal_systick_get_ms_since)(uint32_t tick);
} tHalTransmitCtr;

typedef struct eHalTransmitCallback
{
	tHalKmdMsgType type_transmit; //数据发送类型
	tHalKmdMsgType type_debug;	  //波形发送类型
	//兼容三种通信方式
	tHalTransmitCtr hal_can_;
	tHalTransmitCtr hal_uart;
	tHalTransmitCtr hal_usb_;
} tHalTransmitCallback;

typedef struct
{
	// 复位系统
	void (*const hal_system_reset)(void);
	// Systick
	uint64_t (*const hal_systick_get_us_tick_)(void);
	uint64_t (*const hal_systick_get_us_since)(uint64_t tick_us);
	uint64_t (*const hal_systick_get_ms_tick_)(void);
	uint64_t (*const hal_systick_get_ms_since)(uint64_t tick_ms);
	void (*const hal_systick_delay_ms)(uint64_t ms);
	void (*const hal_systick_delay_us)(uint64_t us);
	// Flash
	int (*const hal_flash_write)(const uint32_t address, uint32_t *const pData, uint32_t data_num, uint32_t page_num);
	int (*const hal_flash_read_)(const uint32_t address, uint32_t *const pData, uint32_t data_num);
	// SVPWM
	void (*const hal_svpwm_init)(void);
	void (*const hal_svpwm_apply_duty)(uint16_t duty_a, uint16_t duty_b, uint16_t duty_c);
	void (*const hal_svpwm_switch_off)(void);
	void (*const hal_svpwm_switch_on_)(void);

	// LED状态控制
	void (*const hal_led_set)(uint32_t timer, uint32_t brightCount);
	// Encoder
	bool (*const hal_encoder_read_raw)(int encoder_type, uint32_t *pReadRaw);
	// 初始化板载接口
	void (*const hal_board_init)(void);
	uint32_t (*const hal_get_mReceiveCycle_ms)(tHalTransmitCallback *pHalTransmitCallback, hal_frame_struct *phal_frame_struct);
	// 发送消息
	bool (*const hal_transmit_fun)(tHalTransmitCallback *pHalTransmitCallback, hal_frame_struct *phal_frame_struct);
	bool (*const hal_rx_transmit_fun)(tHalTransmitCallback *pHalTransmitCallback, hal_frame_struct *phal_frame_struct);
	// 打印数据波形
	bool (*const hal_port_fun)(tHalTransmitCallback *pHalTransmitCallback, uint16_t frameID, float *pData, uint16_t len);

	void (*const hal_get_device_id)(uint8_t *device_id);

	tHalTransmitCallback hal_transmit_type;
} tHalFocInterface;

static inline uint32_t hal_cpu_enter_critical(void)
{

	uint32_t primask;
#ifdef HAL_GD32_LIB_USED
	primask = __get_PRIMASK();
	__disable_irq();
#endif
	return primask;
}

static inline void hal_cpu_exit_critical(uint32_t priority_mask)
{
#ifdef HAL_GD32_LIB_USED
	__set_PRIMASK(priority_mask);
#endif
}

// 初始化
void hal_foc_init_main(void);

// foc检查，放入循环
void hal_foc_check_loop(void);

// mcu硬件错误
bool hal_report_err_hanlder(void);


#endif
