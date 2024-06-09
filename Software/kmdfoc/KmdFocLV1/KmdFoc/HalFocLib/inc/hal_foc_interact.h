

#ifndef _HAL_FOC_INTERACT_H_
#define _HAL_FOC_INTERACT_H_

#include <hal_foc_struct.h>


// 报告错误状态
bool hal_report_state_error(tHalFocInterface  *const pCallbackFun,
							uint32_t reportCanId,
							uint32_t ecode);

// 报告校准返回
bool hal_report_calibration(tHalFocInterface  *const pCallbackFun,
							uint32_t reportCanId,
							uint32_t reportId, 
							uint8_t *data);

// 状态检查，心跳数据上报
void hal_check_loop(tHalFocStruct *const pHalFoc);

// 接收消息入口
bool hal_receive_callback(tHalFocStruct *pHalFoc, hal_frame_struct *rx_frame);

#endif /* __CAN_H__ */
