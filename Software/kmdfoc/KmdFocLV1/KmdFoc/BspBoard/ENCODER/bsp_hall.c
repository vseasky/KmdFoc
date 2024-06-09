#include "bsp_hall.h"

#define tB0001  (1<<0)|(0<<1)|(0<<2) //1
#define tB0011  (1<<0)|(1<<1)|(0<<2) //3
#define tB0010  (0<<0)|(1<<1)|(0<<2) //2
#define tB0110  (0<<0)|(1<<1)|(1<<2) //6
#define tB0100  (0<<0)|(0<<1)|(1<<2) //4
#define tB0101  (1<<0)|(0<<1)|(1<<2) //5

//５－１－３－２－６－４
static uint8_t bsp_hall_read(void);


/**
 * @description: 初始化HALL编码器接口
 * @return {*}
 */
void bsp_hall_init(void)
{
    //(A-PB7)\(B-P6)\(C-P5)
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ,GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5);
    rcu_periph_clock_enable(RCU_GPIOB);
}

/**
 * @description: 读取HALL编码器的值
 * @return {*}
 */
static uint8_t bsp_hall_read(void)
{
    uint8_t hall_state_ = 0;
    hall_state_=
        (	(gpio_input_bit_get(GPIOB,GPIO_PIN_7)<<0)|
            (gpio_input_bit_get(GPIOB,GPIO_PIN_6)<<1)|
            (gpio_input_bit_get(GPIOB,GPIO_PIN_5)<<2));
    for (uint8_t i = 0; i < 45; i++);
    return hall_state_;
}

/**
 * @description: 读取HALL编码器的值
 * @param {uint32_t} *pReadRaw
 * @return {*}
 */
bool bsp_hall_read_raw(uint32_t *pReadRaw)
{
    static uint8_t raw;
    switch (bsp_hall_read())
        {
			case (tB0001):{raw =  0;};break;
			case (tB0011):{raw =  1;};break;
			case (tB0010):{raw =  2;};break;
			case (tB0110):{raw =  3;};break;
			case (tB0100):{raw =  4;};break;
			case (tB0101):{raw =  5;};break;
			default:{}break;
        }
	*pReadRaw = raw;
	return true;
}
