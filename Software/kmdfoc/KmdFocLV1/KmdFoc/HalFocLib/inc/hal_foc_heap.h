
#ifndef _HAL_FOC_HEAP_H__
#define _HAL_FOC_HEAP_H__

//标准头文件
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
//数学库支持
#include <math.h>

#define TOTAL_HEAP_SIZE		((size_t)(1024*16))		// byte

void *HEAP_malloc(size_t xWantedSize);
void HEAP_free(void *pv);
size_t HEAP_get_free_size(void);
size_t HEAP_get_minimumEver_free_size(void);

#endif
