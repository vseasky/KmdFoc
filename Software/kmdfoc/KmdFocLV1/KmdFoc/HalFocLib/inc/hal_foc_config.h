#ifndef _HAL_FOC_CONFIG_H_
#define _HAL_FOC_CONFIG_H_

//标准头文件
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
//数学库支持
#include <math.h>
#include <hal_foc_util.h>

#define HAL_FW_VERSION_1 2
#define HAL_FW_VERSION_2 1
#define HAL_FW_VERSION_3 0
#define HAL_FW_VERSION_4 0

#define HAL_OFFSET_LUT_NUM (128)

#define HAL_SHUNT_RESISTENCE (0.001f)
#define HAL_V_SCALE_DEF (19.0f * 3.3f / 4096.0f)
#define HAL_I_SCALE_DEF ((3.3f / 4096.0f) / HAL_SHUNT_RESISTENCE / 30.482f)
#define HAL_MAX_CURRENT_DEF (HAL_I_SCALE_DEF *(4096.0f) / 2)*0.91           // 理论可测量电流

#define HAL_PWM_ARR_DEF (3000)                  //20KHz
#define HAL_DT (1.0f / 20000.0f)
#define HAL_CURRENT_MEASURE_HZ (20000.0f)
#define HAL_CURRENT_FILT_ALPHA_DEF (0.1f)
#define HAL_VBUS_FILT_ALPHA_DEF (0.1f)

// CALIB
#define HAL_CALB_SPEED M_2PI                     // 校准时的速度 rad/s
#define HAL_MOTOR_POLE_PAIRS_MAX 30.0f           // 支持最大极对数
#define HAL_SAMPLES_PER_PPAIR HAL_OFFSET_LUT_NUM // 偏移补偿

#define HAL_ZERO_CALIB_TIME (0.010f)
#define HAL_ZERO_CALIB_MAX_COUNT (HAL_ZERO_CALIB_TIME) * (HAL_CURRENT_MEASURE_HZ)

#endif
