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
 * @FilePath     : \MDK-ARMe:\KmdFoc\Tools\UsbCanBus\software\CanUsb-V4\CanUsb\CanBus\inc\bsp_can.h
 * @Description  : 
 */

#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#include <stdint.h>
#include <stdbool.h>
#include "gd32c10x.h"



/** 
 * @Description  : can0 初始化 
 * @return        (*)
 */
void bsp_can0_init(void);

/** 
 * @Description  : can0 发送函数 
 * @param         (uint32_t) frameID
 * @param         (uint8_t) *pData
 * @param         (uint8_t) len
 * @return        (*)
 */
bool bsp_can0_transmit(uint32_t frameID, uint8_t *pData, uint8_t len);

/** 
 * @Description  : can0 中断回调 
 * @return        (*)
 */
void bsp_can0_irq_callback(void);

/** 
 * @Description  : can1 初始化 
 * @return        (*)
 */
void bsp_can1_init(void);

/** 
 * @Description  : can1 发送函数
 * @param         (uint32_t) frameID
 * @param         (uint8_t) *pData
 * @param         (uint8_t) len
 * @return        (*)
 */
bool bsp_can1_transmit(uint32_t frameID, uint8_t *pData, uint8_t len);

/** 
 * @Description  : can1 中断回调 
 * @return        (*)
 */
void bsp_can1_irq_callback(void);

#endif