#ifndef _mHalFocInfo_H_
#define _mHalFocInfo_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "hal_foc_struct.h"
#include "hal_kmd_foc.h"

bool kmd_interface_transmit(hal_frame_struct *pFrame);
bool kmd_interface_receive_callback(hal_frame_struct *pFrame);


extern tHalFocInfo mHalFocInfo1;
extern tHalFocInfo mHalFocInfo2;
extern hal_kmd_user_struct tHalKmdUser;
extern const uint8_t KMD_CONFIG_MAP_MAX_LENGTH;
extern const uint8_t kmd_config_map[][2];
#endif