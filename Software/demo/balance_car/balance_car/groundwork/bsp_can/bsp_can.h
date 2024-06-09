#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_


#include "stdint.h"



void can_filter_init(void);

void CAN2_transmit(uint16_t can_id,uint8_t *pData,uint16_t DataLen);


#endif
