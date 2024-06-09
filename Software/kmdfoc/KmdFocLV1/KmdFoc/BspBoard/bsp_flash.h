#ifndef _BSP_FLASH_H_
#define _BSP_FLASH_H_

#include <stdint.h>
#include <stdbool.h>



//GD32C1x 128个闪存页,闪存页大小为1KB。主存储闪存的每页都可以单独擦除。
#define BSP_FMC_PAGE_SIZE 		((uint32_t)0x400U)	
#define BSP_FMC_PAGE_NUM		(128)

#define BSP_FMC_PAGE_MIN		((uint32_t)(0x8000000 + 100 * FMC_PAGE_SIZE))
#define BSP_FMC_PAGE_MAX		((uint32_t)(0x8000000 + BSP_FMC_PAGE_NUM * FMC_PAGE_SIZE))


/**
 * @description: Flash写函数
 * @param {uint32_t} address
 * 				->>读取地址
 * @param {uint32_t} *pData
 * 				->>待写入数据
 * @param {uint32_t} data_num
 * 				->>写入FLASH的页码
 * @param {uint32_t} page_num
 * 				->>需要写入的页码数
 * @return {*}
 */
int bsp_flash_write(const uint32_t address,uint32_t *const pData,uint32_t data_num,uint32_t page_num);


/**
 * @description: FLASH读取函数
 * @param {uint32_t} address
 * 				->>flash 地址
 * @param {uint32_t} *pData
 * 				->>数据地址
 * @param {uint32_t} data_num
 * 				->>需要读取的数据大小
 * @return {*}
 */
int bsp_flash_read_(const uint32_t address,uint32_t *const pData,uint32_t data_num);


#endif
