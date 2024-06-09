#include "bsp_ma730.h"

#define BSP_MA730_NCS_SET() GPIO_BOP(GPIOA) = (uint32_t)GPIO_PIN_4;
#define BSP_MA730_NCS_RESET() GPIO_BC(GPIOA) = (uint32_t)GPIO_PIN_4;

/**
 * @description:初始化编码器接口
 * @return {*}
 */
void bsp_ma730_init(void)
{
    uint32_t raw = 0;
    spi_parameter_struct spi_init_struct;

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_SPI0);

    // PA4   ------> ENC_nCS
    gpio_bit_set(GPIOA, GPIO_PIN_4);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);

    /* SPI1 GPIO config: SCK/PA5, MISO/PA6, MOSI/PA7 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    /* deinitilize SPI and the parameters */
    spi_i2s_deinit(SPI0);
    spi_struct_para_init(&spi_init_struct);

    /* SPI1 parameter config */
    spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode = SPI_MASTER;
    spi_init_struct.frame_size = SPI_FRAMESIZE_16BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_init_struct.nss = SPI_NSS_SOFT;
    spi_init_struct.prescale = SPI_PSC_4; // 30M
    spi_init_struct.endian = SPI_ENDIAN_MSB;
    spi_init(SPI0, &spi_init_struct);

    spi_enable(SPI0);
    
    BSP_MA730_NCS_SET();
    for(uint32_t i=0;i<256;i++);
    bsp_ma730_read_raw(&raw);
}

/**
 * @description: 读取MA730编码器角度
 * @param {uint32_t} *pReadRaw
 * @return {*}
 */
bool bsp_ma730_read_raw(uint32_t *pReadRaw)
{
    int ret = 0;
    uint16_t rxData = 0;
    uint32_t timeout_cnt;
    rxData = 0;
    const uint32_t timeout_cnt_num = 10000;
    // CS
    BSP_MA730_NCS_RESET();
    {
        ret = 0;

        timeout_cnt = 0;
        while ((RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)) && (ret == 0))
        {
            timeout_cnt++;
            if (timeout_cnt > timeout_cnt_num)
            {
                ret = -1;
                break;
            }
        }
        spi_i2s_data_transmit(SPI0, BSP_MA730_READ);
        for (uint8_t i = 0; i < 25; i++);
        timeout_cnt = 0;
        while ((RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE)) && (ret == 0))
        {
            timeout_cnt++;
            if (timeout_cnt > timeout_cnt_num)
            {
                ret = -1;
                break;
            }
        }
        rxData = spi_i2s_data_receive(SPI0);
        for (uint8_t i = 0; i < 15; i++);
    }
    // NCS
    BSP_MA730_NCS_SET();

    if (ret == 0)
    {
        *pReadRaw = (rxData >> 2);
        return (true);
    }
    else
    {
        return false;
    }
}
