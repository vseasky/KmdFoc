#ifndef _HAL_FOC_CALIBRATION_H__
#define _HAL_FOC_CALIBRATION_H__

#include <hal_foc_struct.h>


// 开始校准
void hal_calibration_start(tHalCalibStruct *pCalib);

// 结束校准
void hal_calibration_end(tHalCalibStruct *pCalib);

// 校准循环
void hal_calibration_loop(tHalFocStruct *pHalFoc);

#endif
