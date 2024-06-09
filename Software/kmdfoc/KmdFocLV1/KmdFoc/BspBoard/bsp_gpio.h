#ifndef _BSP_GPIO_H_
#define _BSP_GPIO_H_

#include "bsp_interface.h"
#include "gd32c10x.h"
#define LED_RCU_GROUP 	RCU_GPIOC
#define LED_GROUP 		GPIOC
#define LED_Pin   		GPIO_PIN_13



#define LED_BRIGHT			0				//LEDÁÁÆðµçÆ½

static inline void bsp_led_set(uint32_t state)
{
#if LED_BRIGHT == 1 
	if (state)
	{
		GPIO_BOP(LED_GROUP) = (uint32_t)LED_Pin;
	}
	else
	{
		GPIO_BC(LED_GROUP) = (uint32_t)LED_Pin;
	}
#else
	if (!state)
	{
		GPIO_BOP(LED_GROUP) = (uint32_t)LED_Pin;
	}
	else
	{
		GPIO_BC(LED_GROUP) = (uint32_t)LED_Pin;
	}
	
#endif
}

static inline uint32_t bsp_led_get(void)
{
#if LED_BRIGHT == 1 
	return (GPIO_OCTL(LED_GROUP) & (LED_Pin));
#else
	return !(GPIO_OCTL(LED_GROUP) & (LED_Pin));
#endif
}

static inline void bsp_leg_toggle(void)
{
	if (bsp_led_get())
	{
		bsp_led_set(0);
	}
	else
	{
		bsp_led_set(1);
	}
}

/**
 * @description: ???LED
 * @return {*}
 */
void bsp_led_init(void);


/**
 * @description: led??????
 * @return {*}
 */
void bsp_led_callback(void);

/**
 * @description: LED????
 * @param {uint32_t} timer
 * @param {uint32_t} brightCount
 * @return {*}
 */
void bsp_led_state_set(uint32_t timer,uint32_t brightCount);


#endif
