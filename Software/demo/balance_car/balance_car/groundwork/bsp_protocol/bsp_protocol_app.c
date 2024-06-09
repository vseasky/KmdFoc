#include "bsp_protocol_app.h"
#include "usbd_cdc_if.h"

extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

protocol_struct mTxProtocol;
protocol_struct mRxProtocol;
void mx_protocol_init(void)
{
	init_protocol(&mTxProtocol,24);
	init_protocol(&mRxProtocol,24);
}

void protocol_debug(uint32_t CmdId,float *pData,uint32_t uLen)
{
	mTxProtocol.frame_st.header.sof = 0XA5;
	mTxProtocol.frame_st.frame_user.equipment_type = 0x01;
	mTxProtocol.frame_st.frame_user.equipment_id = 0x01;
	mTxProtocol.frame_st.frame_user.data_id = CmdId;
	mTxProtocol.frame_st.frame_user.cmd_data.data_len =  uLen;
	memcpy(mTxProtocol.frame_st.frame_user.cmd_data.pData,(uint32_t*)pData,uLen*4);
	make_protocol(&mTxProtocol);
	memcpy(UserTxBufferFS,mTxProtocol.message_st.pData,mTxProtocol.message_st.data_len);
	CDC_Transmit_FS(mTxProtocol.message_st.pData,mTxProtocol.message_st.data_len);
}