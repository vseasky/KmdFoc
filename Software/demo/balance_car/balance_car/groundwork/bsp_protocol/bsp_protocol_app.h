#ifndef _BSP_PROTOCOL_APP_H_
#define _BSP_PROTOCOL_APP_H_
#include "bsp_protocol.h"


void mx_protocol_init(void);
void protocol_debug(uint32_t CmdId,float *pData,uint32_t uLen);


#endif