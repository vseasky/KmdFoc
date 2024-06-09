#include "bsp_device_id.h"

//#define BSP_DEVICE_ID1                (0x1FFFF7E8U)         /* device ID1 0x1FFFF7E8*/
//#define BSP_DEVICE_ID2                (0x1FFFF7ECU)         /* device ID2 0x1FFFF7EC*/
//#define BSP_DEVICE_ID3                (0x1FFFF7F0U)         /* device ID3 0x1FFFF7F0*/

void bsp_device_id_get(uint8_t *device_id)
{
    memcpy(&device_id[0],(uint8_t*)(0x1FFFF7E8U),12);
}
