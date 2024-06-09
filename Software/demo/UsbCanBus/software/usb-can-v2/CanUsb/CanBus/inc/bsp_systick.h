/** 
 * @License      : 
 * @
 * @	<one line to give the program's name and a brief idea of what it does.>
 * @	Copyright (C) 2022  vSeasky.Liu liuwei_seasky@163.com
 * @	This program is free software: you can redistribute it and/or modify
 * @	it under the terms of the GNU General Public License as published by
 * @	the Free Software Foundation, either version 3 of the License, or
 * @	(at your option) any later version.
 * @
 * @	This program is distributed in the hope that it will be useful,
 * @	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * @	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * @	GNU General Public License for more details.
 * @
 * @	You should have received a copy of the GNU General Public License
 * @	along with this program.  If not, see <https://www.gnu.org/licenses/>.
 * @
 * @Author       : Copyright (c) 2022, vSeasky.Liu liuwei_seasky@163.com.
 * @Github       : https://github.com/SEASKY-Master
 * @Date         : 2022-05-27 14:01:11
 * @FilePath     : \MDK-ARMe:\KmdFoc\Tools\UsbCanBus\software\CanUsb-V4\CanUsb\CanBus\inc\bsp_systick.h
 * @Description  : 
 */
#ifndef _BSP_SYSTICK_H__
#define _BSP_SYSTICK_H__

#include "main.h"



extern volatile uint32_t SysTickCnt;

/**
 * @description: 初始化Systick
 * @return {*}
 */
void bsp_systick_init(void);

/**
 * @description: 获取系统时间us
 * @return {*}
 */
uint64_t bsp_systick_get_tick_us(void);

/**
 * @description: 获取系统时间差us
 * @param {uint64_t} tick_us
 * @return {*}
 */
uint64_t bsp_systick_get_us_since(uint64_t tick_us);

/**
 * @description: 获取系统时间ms
 * @return {*}
 */
uint32_t bsp_systick_get_tick_ms(void);


/**
 * @description: 计算系统时间差ms
 * @param {uint32_t} tick
 * @return {*}
 */
uint32_t bsp_systick_get_ms_since(uint32_t tick_ms);

/**
 * @description: systick us延时函数
 * @param {uint32_t} uS
 * @return {*}
 */
void bsp_systick_delay_us(uint32_t us);


/**
 * @description: ms延时函数
 * @param {uint32_t} mS
 * @return {*}
 */
void bsp_systick_delay_ms(uint32_t ms);


#endif
