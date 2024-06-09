#ifndef _BSP_MT6825_H_
#define _BSP_MT6825_H_

#include "main.h"


#define BSP_MT6825_READ_	   0X80
#define BSP_MT6825_WRITE	   0X00

#define BSP_MT8825_ANGLE_17_10 0X03 //ANGLE[17:10]
#define BSP_MT8825_ANGLE_09_04 0X04	//ANGLE[09:04]
#define BSP_MT8825_ANGLE_03_00 0X04	//ANGLE[03:00]

#define BSP_MT6825_ENCODER_CPR_BIT 	18										//18BIT
#define BSP_MT6825_ENCODER_CPR		(1<<BSP_MT6825_ENCODER_CPR_BIT) 		//18BIT

/**
 * @description: 初始化mt6825编码器接口
 * @return {*}
 */
void bsp_mt6825_init(void);


/**
 * @description: 读取mt6825编码器角度
 * @param {uint32_t} *pReadRaw
 * @return {*}
 */
bool bsp_mt6825_read_raw(uint32_t *pReadRaw);

#endif
