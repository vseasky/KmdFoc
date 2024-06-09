#include "bsp_can.h"
#include "main.h"

#include "hal_kmd_interface.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

void CAN2_transmit(uint16_t can_id,uint8_t *pData,uint16_t DataLen)
{
	CAN_TxHeaderTypeDef  transmit_message;
    uint32_t send_mail_box;
    transmit_message.StdId = can_id;
    transmit_message.IDE = CAN_ID_STD;
    transmit_message.RTR = CAN_RTR_DATA;
    transmit_message.DLC = 0x08;
	while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan2 )==0);
    HAL_CAN_AddTxMessage(&hcan2, &transmit_message, pData, &send_mail_box);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	hal_frame_struct mFrame;
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header,&mFrame.data[0]);
	mFrame.can_id = rx_header.StdId;
	mFrame.data_len = 8 ;
	kmd_interface_receive_callback(&mFrame);
}

