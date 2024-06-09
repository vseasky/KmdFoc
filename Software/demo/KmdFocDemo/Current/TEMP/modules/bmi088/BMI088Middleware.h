/*
 * @Author: seasky 2544907929@qq.com
 * @Date: 2022-05-27 13:19:37
 * @LastEditors: seasky 2544907929@qq.com
 * @LastEditTime: 2022-06-25 09:18:40
 * @FilePath: \TEMP\TEMP\modules\bmi088\BMI088Middleware.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "struct_typedef.h"

#define BMI088_USE_SPI


void bsp_bmi088_gpio_init(void);
void bsp_bmi088_com_init(void);
uint64_t bsp_bmi088_get_us(void);
void bsp_bmi088_delay_ms(uint16_t ms);
void bsp_bmi088_delay_us(uint16_t us);

extern void BSP_BMI088_ACCEL_NS_L(void);
extern void BSP_BMI088_ACCEL_NS_H(void);

extern void BSP_BMI088_GYRO_NS_L(void);
extern void BSP_BMI088_GYRO_NS_H(void);

extern uint8_t BSP_BMI088_Read_Write_Byte(uint8_t reg);



#endif
