#ifndef BSP_ADC_H
#define BSP_ADC_H
#include "struct_typedef.h"
void init_vrefint_reciprocal(void);
float get_temprate(void);
float get_battery_voltage(void);
#endif
