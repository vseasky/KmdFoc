#include "hal_kmd_interface.h"
#include "hal_foc_struct.h"
#include "hal_kmd_foc.h"

#include "bsp_can.h"
#include "bsp_kmd.h"
tHalFocInfo mHalFocInfo[5]
=
{
    [0].node_id = 1,
    [1].node_id = 2,
    [2].node_id = 6,
    [3].node_id = 5,
};

//消息发送接口函数
bool kmd_interface_transmit(hal_frame_struct *pFrame)
{
    bsp_can_can_transmit(pFrame);
}
//消息接收接口函数
bool kmd_interface_receive_callback(hal_frame_struct *pFrame)
{
    hal_frame_struct mFrame[4];
    memcpy(&mFrame[0],pFrame,sizeof(hal_frame_struct));
    memcpy(&mFrame[1],pFrame,sizeof(hal_frame_struct));
    memcpy(&mFrame[2],pFrame,sizeof(hal_frame_struct));
    hal_kmd_frame_receive_callback(pFrame,&mHalFocInfo[0]);
    hal_kmd_frame_receive_callback(&mFrame[0],&mHalFocInfo[1]);
    hal_kmd_frame_receive_callback(&mFrame[1],&mHalFocInfo[2]);
    hal_kmd_frame_receive_callback(&mFrame[2],&mHalFocInfo[3]);
}


