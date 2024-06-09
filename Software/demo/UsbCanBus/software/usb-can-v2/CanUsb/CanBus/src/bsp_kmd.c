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
 * @FilePath     : \MDK-ARMe:\KmdFoc\Tools\UsbCanBus\software\CanUsb-V4\CanUsb\CanBus\src\bsp_kmd.c
 * @Description  :
 */
#include "bsp_kmd.h"
#include "main.h"

kmd_usb kmd_usb_t;

// 外部预先分配好内存空间
uint32_t pTxData[PROTOCOL_MAX_U32_LEN];
uint8_t pTxBuffer[PROTOCOL_MAX_U32_LEN * 4 + PROTOCOL_DATA_OFFSET + 2];
uint32_t pRxData[64];
uint8_t pRxBuffer[PROTOCOL_MAX_U32_LEN * 4 + PROTOCOL_DATA_OFFSET + 2];

/** 
 * @Description  :  kmd 消息解析，转换为can数据帧格式
 * @param         (tHalKmdMsgType) pMsgType
 * @return        (*)
 */
static bool bsp_usb_can_parse(protocol_struct *pProtocol, tHalKmdMsgType pMsgType);

/** 
 * @Description  : 对于USB转CAN控制KmdFoc直接转发消息
 * @param         (bsp_frame) *rx_frame
 * @return        (*)
 */
static bool bsp_usb_can_transmit(bsp_frame *rx_frame);

/** 
 * @Description  : 初始化协议所需内存
 * @return        (*)
 */
void bsp_protocol_init()
{
    kmd_usb_t.pTxProtocol.frame_st.frame_user.cmd_data.pData = NULL;
    kmd_usb_t.pTxProtocol.message_st.pData = NULL;
    init_protocol_pointer(&kmd_usb_t.pTxProtocol, pTxData, pTxBuffer, PROTOCOL_MAX_U32_LEN);
    kmd_usb_t.pRxProtocol.frame_st.frame_user.cmd_data.pData = NULL;
    kmd_usb_t.pRxProtocol.message_st.pData = NULL;
    init_protocol_pointer(&kmd_usb_t.pRxProtocol, pRxData, pRxBuffer, PROTOCOL_MAX_U32_LEN);
}




/**
 * @Description  : usb消息接收，数据解析
 * @param         (uint8_t) *pData
 * @param         (uint16_t) uLen
 * @param         (tHalKmdMsgType) pMsgType
 * @return        (*)
 */
bool bsp_usb_can_callback(uint8_t *pData, uint16_t uLen, tHalKmdMsgType pMsgType)
{
    uint16_t uPos;
    if ((uLen > 0) && (uLen <= kmd_usb_t.pRxProtocol.message_st.max_data_len))
    {
        memcpy(pRxBuffer, pData, uLen);
        parse_protocol(&kmd_usb_t.pRxProtocol, uLen);
        //如果未定义USB转CAN，按同种方式返回
        switch (kmd_usb_t.pRxProtocol.frame_st.frame_user.equipment_type)
        {
        case BSP_EQ_TYPE_NULL:;
            break;
            // 为 kmdFoc设备
        case BSP_EQ_TYPE_KMD_FOC:
        {
            // 如果是 BSP_EQ_TYPE_KMD_FOC消息
            return bsp_usb_can_parse(&kmd_usb_t.pRxProtocol, pMsgType);
        }
        break;
        default:;
            break;
        }
    }
    return false;
}

/** 
 * @Description  :  usb 消息解析，转换为can数据帧格式
 * @param         (tHalKmdMsgType) pMsgType
 * @return        (*)
 */
static bool bsp_usb_can_parse(protocol_struct *pProtocol, tHalKmdMsgType pMsgType)
{
    bsp_frame frame;
    frame.can_dlc = 0;
    frame.msg_type = pMsgType;
    frame.can_id = pProtocol->frame_st.frame_user.equipment_id;
    memcpy(frame.data, pProtocol->frame_st.frame_user.cmd_data.pData, 8);
    return bsp_usb_can_transmit(&frame);
}

/** 
 * @Description  : 对于USB转CAN控制KmdFoc直接转发消息
 * @param         (bsp_frame) *rx_frame
 * @return        (*)
 */
static bool bsp_usb_can_transmit(bsp_frame *rx_frame)
{
#if (defined(KMD_FOC_USB_CAN)) && (KMD_FOC_USB_CAN == 0)
    return bsp_can0_transmit(rx_frame->can_id, rx_frame->data, 8);
#else 
    return bsp_can1_transmit(rx_frame->can_id, rx_frame->data, 8);
#endif
    return false;
}

/** 
 * @Description  : 一路用于USB转CAN，另一路做普通功能使用，普通功能can发送函数接口
 * @param         (bsp_frame) *rx_frame
 * @return        (*)
 */
bool bsp_can_can_transmit(bsp_frame *rx_frame)
{
#if (defined(KMD_FOC_USB_CAN)) && (KMD_FOC_USB_CAN == 0)
        return bsp_can1_transmit(rx_frame->can_id, rx_frame->data, 8);
#else 
        return bsp_can0_transmit(rx_frame->can_id, rx_frame->data, 8);
    
#endif
}



/** 
 * @Description  : can接收can发送
 * @param         (bsp_frame) *rx_frame
 * @return        (*)
 */
bool bsp_can_usb_transmit(bsp_frame *tx_frame)
{
    const uint8_t dataLen = 8;
    kmd_usb_t.pTxProtocol.frame_st.frame_user.equipment_type = BSP_EQ_TYPE_KMD_FOC;
    kmd_usb_t.pTxProtocol.frame_st.frame_user.equipment_id = (tx_frame->can_id);
    kmd_usb_t.pTxProtocol.frame_st.frame_user.data_id = 0;
    kmd_usb_t.pTxProtocol.frame_st.frame_user.cmd_data.data_len = dataLen / 4;
    memcpy((char *)(kmd_usb_t.pTxProtocol.frame_st.frame_user.cmd_data.pData), (char *)tx_frame->data, dataLen);
    make_protocol(&kmd_usb_t.pTxProtocol);
#ifdef KMD_USB_USER
    return bsp_usb_transmit((uint8_t *)(kmd_usb_t.pTxProtocol.message_st.pData), kmd_usb_t.pTxProtocol.message_st.data_len);
#endif
}


