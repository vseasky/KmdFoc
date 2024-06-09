#ifndef _HAL_FOC_USR_CONFIG_H_
#define _HAL_FOC_USR_CONFIG_H_

#include <hal_foc_struct.h>

// 设置默认参数
void hal_usr_set_default_config(tHalFocUser *const pFocUser);

// 从FLASH读取参数
int hal_usr_read_config(tHalFocUser * const pFocUser);

// 保存参数到FLASH
int hal_usr_save_config(tHalFocUser * const pFocUser);


#endif
