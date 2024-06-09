#ifndef _HAL_FOC_ENCODER_H_
#define _HAL_FOC_ENCODER_H_

#include <hal_foc_struct.h>


// 编码器参数清零&复位
void hal_encoder_reset_def(tHalEncoderStruct *pEncoder);


// 编码器角度获取，计算
void hal_encoder_sample(tHalEncoderStruct *pEncoder);

#endif
