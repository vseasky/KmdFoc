#include "bsp_encoder.h"
#include "hal_foc_encoder.h"
#include "hal_foc_usr_config.h"

/**
 * @description: 初始化编码器
 * @param {int} encoder_type
 * 				->>编码器类型
 * @return {*}
 */
void bsp_encoder_init(int encoder_type)
{
	switch(encoder_type)
	{
		case HAL_ENCODER_MT6825:bsp_mt6825_init();;break;
		case HAL_ENCODER_MA730 :bsp_ma730_init();;break;
		case HAL_ENCODER_AS5047P:bsp_as5047p_init();break;
		case HAL_ENCODER_HALL  :bsp_hall_init();;break;
	}
}


/**
 * @description: 读取编码器的值
 * @param {int} encoder_type
 * @param {uint32_t} *pReadRaw
 * 				->>编码器类型
 * @return {*}
 */
bool bsp_encoder_read_raw(int encoder_type,uint32_t *pReadRaw)
{
	switch(encoder_type)
	{
		case HAL_ENCODER_MT6825 :{return bsp_mt6825_read_raw(pReadRaw);};break;
		case HAL_ENCODER_MA730	:{return bsp_ma730_read_raw(pReadRaw);};break;
		case HAL_ENCODER_AS5047P:{return bsp_as5047p_read_raw(pReadRaw);};break;
		case HAL_ENCODER_HALL	:{return bsp_hall_read_raw(pReadRaw);};break;
	}
	return false;
}
