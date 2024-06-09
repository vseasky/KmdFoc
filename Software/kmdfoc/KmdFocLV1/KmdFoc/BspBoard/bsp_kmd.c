#include "bsp_kmd.h"
#include "bsp_interface.h"
#include "main.h"

kmd_usb kmd_usb_t;
// 分配seasky协议数据处理所需内存
uint32_t pTxData[KMD_U32_LEN];
uint8_t  pTxBuffer[KMD_U32_LEN * 4 + PROTOCOL_DATA_OFFSET+2];
uint32_t pRxData[KMD_U32_LEN];
uint8_t  pRxBuffer[KMD_U32_LEN * 4 + PROTOCOL_DATA_OFFSET+2];
uint32_t pDebTxData[KMD_DEBUG_U32_LEN];
uint8_t  pDebTxBuffer[KMD_DEBUG_U32_LEN * 4 + PROTOCOL_DATA_OFFSET+2];

bool bsp_kmd_foc_callback(protocol_struct *pProtocol,BSP_KMD_COM_TYPE pMsgType);

/**
 * @description: 初始化Seasky协议所需内存，手动分配内存
 * @return {*}
 */
void bsp_kmd_init(void)
{
    kmd_usb_t.pTxProtocol.frame_st.frame_user.cmd_data.pData = NULL;
    kmd_usb_t.pTxProtocol.message_st.pData = NULL;
    init_protocol_pointer(&kmd_usb_t.pTxProtocol, pTxData, pTxBuffer, KMD_U32_LEN);
    kmd_usb_t.pRxProtocol.frame_st.frame_user.cmd_data.pData = NULL;
    kmd_usb_t.pRxProtocol.message_st.pData = NULL;
    init_protocol_pointer(&kmd_usb_t.pRxProtocol, pRxData, pRxBuffer, KMD_U32_LEN);
	kmd_usb_t.pDebTxProtocol.frame_st.frame_user.cmd_data.pData = NULL;
    kmd_usb_t.pDebTxProtocol.message_st.pData = NULL;
    init_protocol_pointer(&kmd_usb_t.pDebTxProtocol, pDebTxData, pDebTxBuffer, KMD_DEBUG_U32_LEN);
}


/**
 * @description: USB和串口共用接收数据处理函数
 * @param {uint8_t} *pData
 * @param {uint16_t} uLen
 * @param {BSP_KMD_COM_TYPE} pMsgType
 * @return {*}
 */
bool bsp_kmd_callback(uint8_t *pData,uint16_t uLen,BSP_KMD_COM_TYPE pMsgType)
{
    uint16_t uPos;
    if((uLen>0)&&(uLen<=kmd_usb_t.pRxProtocol.message_st.max_data_len))
        {
            memcpy(pRxBuffer,pData,uLen);
            // 进行数据解析
            parse_protocol(&kmd_usb_t.pRxProtocol,uLen);
            switch (kmd_usb_t.pRxProtocol.frame_st.frame_user.equipment_type)
                {
                case BSP_EQ_TYPE_NULL:
                    ;
                    break;
                case BSP_EQ_TYPE_KMD_FOC:
				{
                    return bsp_kmd_foc_callback(&kmd_usb_t.pRxProtocol,pMsgType);
				}
                    break;
                default:
                    ;
                    break;
                }
        }
	return false;
}

/**
 * @description: USB和串口共用接收数据处理函数
 * @param {BSP_KMD_COM_TYPE} pMsgType
 * @return {*}
 */
bool bsp_kmd_foc_callback(protocol_struct *pProtocol,BSP_KMD_COM_TYPE pMsgType)
{
    bsp_frame frame;
    frame.can_dlc = 0;
    frame.msg_type = pMsgType;
    frame.can_id = pProtocol->frame_st.frame_user.equipment_id;
    memcpy(frame.data,pProtocol->frame_st.frame_user.cmd_data.pData,8);
    return bsp_receive_callback(&frame);
}


/**
 * @description: USB和串口共用发送协议数据处理函数
 * @param {BSP_KMD_COM_TYPE} tKmdCom_type
 * @return {*}
 */
bool bsp_kmd_transmit(hal_frame_struct *tx_frame,BSP_KMD_COM_TYPE tKmdCom_type)
{
    const uint8_t dataLen = 8;
    // 设置设备类型为BSP_EQ_TYPE_KMD_FOC
    kmd_usb_t.pTxProtocol.frame_st.frame_user.equipment_type 	= BSP_EQ_TYPE_KMD_FOC;
    // 设置设备ID为 can_id
    kmd_usb_t.pTxProtocol.frame_st.frame_user.equipment_id	 	= tx_frame->can_id;
    // 数据ID 对于Kmdfoc 而言 不使用
    kmd_usb_t.pTxProtocol.frame_st.frame_user.data_id		 	= 0;
    // 设置 数据长度，uint32_t
    kmd_usb_t.pTxProtocol.frame_st.frame_user.cmd_data.data_len = dataLen/4;
    // 赋值待发送数据到协议解析区
    memcpy((char*)(kmd_usb_t.pTxProtocol.frame_st.frame_user.cmd_data.pData),(char*)tx_frame->data,dataLen);
    // 根据SEASKY协议生成发送缓冲
    make_protocol(&kmd_usb_t.pTxProtocol);
    switch(tKmdCom_type)
        {

        case BSP_MSG_FRAME_UART:
		{
            // 串口方式发送消息
            return bsp_uart_transmit((uint8_t*)(kmd_usb_t.pTxProtocol.message_st.pData),kmd_usb_t.pTxProtocol.message_st.data_len);
		}break;
#ifdef KMD_USB_USER
        case BSP_MSG_FRAME_USB:
		{
            // USB方式发送消息
            return bsp_kmd_usb_transmit((uint8_t*)(kmd_usb_t.pTxProtocol.message_st.pData),kmd_usb_t.pTxProtocol.message_st.data_len);
		}break;
#endif
		default:break;
        }
	return false;
}

/**
 * @description: 串口方式打印调试数据
 * @param {uint16_t} frameID
 * @param {float} *pData
 * @param {uint16_t} len
 * @return {*}
 */
bool bsp_kmd_uart_debug(uint16_t frameID,float *pData, uint16_t len)
{
    kmd_usb_t.pDebTxProtocol.frame_st.frame_user.equipment_type 	= BSP_EQ_TYPE_KMD_FOC;
    kmd_usb_t.pDebTxProtocol.frame_st.frame_user.equipment_id	 	= 0x0001;
    kmd_usb_t.pDebTxProtocol.frame_st.frame_user.data_id		 	= frameID;
    kmd_usb_t.pDebTxProtocol.frame_st.frame_user.cmd_data.data_len = len;
    memcpy((char*)(kmd_usb_t.pDebTxProtocol.frame_st.frame_user.cmd_data.pData),(char*)pData,len*4);
    make_protocol(&kmd_usb_t.pDebTxProtocol);
	return bsp_uart_transmit((uint8_t*)(kmd_usb_t.pDebTxProtocol.message_st.pData),kmd_usb_t.pDebTxProtocol.message_st.data_len);
}






