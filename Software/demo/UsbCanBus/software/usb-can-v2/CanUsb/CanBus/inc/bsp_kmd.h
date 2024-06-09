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
 * @FilePath     : \MDK-ARMe:\KmdFoc\Tools\UsbCanBus\software\CanUsb-V4\CanUsb\CanBus\inc\bsp_kmd.h
 * @Description  : 
 */

#ifndef _BSP_KMD_H_
#define _BSP_KMD_H_

#include <stdbool.h>
#include "bsp_protocol.h"
#include "bsp_kmd.h"
#include "hal_kmd_interface.h"


#define PROTOCOL_MAX_U32_LEN 24


typedef hal_frame_struct bsp_frame;

typedef enum
{
	BSP_EQ_TYPE_NULL = 0,
	BSP_EQ_TYPE_KMD_FOC = 1,
}SkEquipmentEnum;

typedef struct
{
	protocol_struct pTxProtocol;// 发送数据帧协议解析或生成
	protocol_struct pRxProtocol;// 接收数据帧协议解析或生成
}kmd_usb;


/** 
 * @Description  : 初始化协议所需内存
 * @return        (*)
 */
void bsp_protocol_init();

/**
 * @Description  : usb消息接收，数据解析
 * @param         (uint8_t) *pData
 * @param         (uint16_t) uLen
 * @param         (tHalKmdMsgType) pMsgType
 * @return        (*)
 */
bool bsp_usb_can_callback(uint8_t *pData,uint16_t uLen,tHalKmdMsgType pMsgType);


/** 
 * @Description  : can数据转换为usb数据发送
 * @param         (bsp_frame) *tx_frame
 * @return        (*)
 */
bool bsp_can_usb_transmit(bsp_frame *tx_frame);


/** 
 * @Description  : can接收can发送
 * @param         (bsp_frame) *rx_frame
 * @return        (*)
 */
bool bsp_can_can_transmit(bsp_frame *rx_frame);


#endif