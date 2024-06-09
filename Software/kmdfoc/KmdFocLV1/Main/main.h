#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "gd32c10x.h"

//#define __DEBUG__

#ifdef __DEBUG__
	#define DEBUG(...)		printf(__VA_ARGS__);
#else
	#define DEBUG(...)
#endif


#ifdef KMD_USB_USER
bool bsp_kmd_usb_transmit(uint8_t *pData,uint16_t len);
#endif

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
