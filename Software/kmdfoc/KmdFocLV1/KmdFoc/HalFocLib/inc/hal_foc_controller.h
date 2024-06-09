#ifndef _HAL_FOC_CONTROLLER_H_
#define _HAL_FOC_CONTROLLER_H_

#include <hal_foc_struct.h>

// 位置模式时设置目标位置
void hal_ctr_move_to_pos(tHalFocStruct *pHalFoc, float goal_point, bool isTraj);

// FOC闭环控制相关参数清零
void hal_ctr_reset(tHalFocStruct *pHalFoc);

// FOC 串级PID计算
tHalFocCtrOut * hal_ctr_loop(tHalFocStruct *pHalFoc, int control_mode);


#endif
