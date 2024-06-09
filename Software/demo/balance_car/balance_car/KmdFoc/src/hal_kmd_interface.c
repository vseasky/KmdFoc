#include "hal_kmd_interface.h"
#include "hal_foc_struct.h"
#include "hal_kmd_foc.h"

#include "bsp_can.h"


tHalFocInfo mHalFocInfo1
=
{
    .node_id = 0X01,
};
tHalFocInfo mHalFocInfo2 =
{
    .node_id = 0X02,
};
//消息发送接口函数
bool kmd_interface_transmit(hal_frame_struct *pFrame)
{
	CAN2_transmit(pFrame->can_id,pFrame->data,pFrame->data_len);
}
//消息接收接口函数
bool kmd_interface_receive_callback(hal_frame_struct *pFrame)
{
    hal_kmd_frame_receive_callback(pFrame,&mHalFocInfo1);
    hal_kmd_frame_receive_callback(pFrame,&mHalFocInfo2);
}

