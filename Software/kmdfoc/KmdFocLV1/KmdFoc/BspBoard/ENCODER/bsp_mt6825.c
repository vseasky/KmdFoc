#include "bsp_mt6825.h"

#define BSP_MT6825_NCS_SET() GPIO_BOP(GPIOA) = (uint32_t)GPIO_PIN_4;
#define BSP_MT6825_NCS_RESET() GPIO_BC(GPIOA) = (uint32_t)GPIO_PIN_4;

/**
 * @description: 初始化mt6825编码器接口
 * @return {*}
 */
void bsp_mt6825_init(void)
{
    uint32_t raw = 0;
    // APB2: Fmax = 120MHz
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
    
    
    BSP_MT6825_NCS_SET();
    for(uint32_t i=0;i<256;i++);
    bsp_mt6825_read_raw(&raw);
}

/**
 * @description: 读取mt6825编码器角度
 * @param {uint32_t} *pReadRaw
 * @return {*}
 */
bool bsp_mt6825_read_raw(uint32_t *pReadRaw)
{
    bool no_mag_warning; // 0X04[1]
    bool over_speed;     // 0X05[3]
    uint16_t u16_data[2];
    uint8_t  u8_data[3];
    int ret = 0;
    uint32_t timeout_cnt;
    const uint32_t timeout_cnt_num = 10000;
    //CS
    BSP_MT6825_NCS_RESET();
    {
        timeout_cnt = 0;
        while ((RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE))&&(ret==0))
            {
                timeout_cnt++;
                if(timeout_cnt>timeout_cnt_num)
                    {
                        ret = -1;
                        break;
                    }
            }
        spi_i2s_data_transmit(SPI0,0x83FF);//transmit1
        for (uint8_t i = 0; i < 25; i++);
        timeout_cnt = 0;
        while ((RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE))&&(ret==0))
            {
                timeout_cnt++;
                if(timeout_cnt>timeout_cnt_num)
                    {
                        ret = -1;
                        break;
                    }
            }
        u16_data[0] = spi_i2s_data_receive(SPI0);//receive1
        for (uint8_t i = 0; i < 15; i++);
        timeout_cnt = 0;
        while ((RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE))&&(ret==0))
            {
                timeout_cnt++;
                if(timeout_cnt>timeout_cnt_num)
                    {
                        ret = -1;
                        break;
                    }
            }
        spi_i2s_data_transmit(SPI0, 0xFFFF);//transmit2
        for (uint8_t i = 0; i < 25; i++);
        timeout_cnt = 0;
        while ((RESET == spi_i2s_flag_get(SPI0,SPI_FLAG_RBNE))&&(ret==0))
            {
                timeout_cnt++;
                if(timeout_cnt>timeout_cnt_num)
                    {
                        ret = -1;
                        break;
                    }
            }
        u16_data[1] = spi_i2s_data_receive(SPI0);//receive2
        for (uint8_t i = 0; i < 15; i++);
    }
    // NCS
    BSP_MT6825_NCS_SET();
    if(ret==0)
        {
            u8_data[0]=u16_data[0]&0XFF;
            u8_data[1]=(u16_data[1]>>8)&0XFF;
            u8_data[2]=u16_data[0]&0XFF;

            // *pReadRaw = 0;
            no_mag_warning 	= (u8_data[1] >> 1) & 0X01;
            over_speed 		= (u8_data[2] >> 3) & 0X01;
            u8_data[1] >>= 2;
            u8_data[2] >>= 4;
            *pReadRaw = (u8_data[0] << 10) | (u8_data[1] << 4) | u8_data[2];
            return (true);
        }
    else
        {
            return (false);
        }
}

//spi transmit receive 8bit
//bool bsp_mt6825_read_raw(uint32_t *pReadRaw)
//{
//	uint8_t data[3];
//	bool no_mag_warning; // 0X04[1]
//	bool over_speed;     // 0X05[3]
//    int ret = 0;
//    uint32_t timeout_cnt;
//    static const uint32_t timeout_cnt_num = 10000;
//    //CS
//    BSP_MT6825_NCS_RESET();
//    {
//		timeout_cnt = 0;
//        while ((RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE))&&(ret==0))
//            {
//                timeout_cnt++;
//                if(timeout_cnt>=timeout_cnt_num)
//                    {
//                        ret = -1;
//                        break;
//                    }
//            }
//        spi_i2s_data_transmit(SPI0, 0x83);//transmit1
//		timeout_cnt = 0;
//        while ((RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE))&&(ret==0))
//            {
//                timeout_cnt++;
//                if(timeout_cnt>=timeout_cnt_num)
//                    {
//                        ret = -1;
//                        break;
//                    }
//            }
//        spi_i2s_data_transmit(SPI0, 0xFF);//transmit2
//		timeout_cnt = 0;
//        while ((RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE))&&(ret==0))
//            {
//                timeout_cnt++;
//                if(timeout_cnt>=timeout_cnt_num)
//                    {
//                        ret = -1;
//                        break;
//                    }
//            }
//        spi_i2s_data_receive(SPI0);//receive1
//		timeout_cnt = 0;
//        while ((RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE))&&(ret==0))
//            {
//                timeout_cnt++;
//                if(timeout_cnt>=timeout_cnt_num)
//                    {
//                        ret = -1;
//                        break;
//                    }
//            }
//        spi_i2s_data_transmit(SPI0, 0xFF);//transmit3
//		timeout_cnt = 0;
//        while ((RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE))&&(ret==0))
//            {
//                timeout_cnt++;
//                if(timeout_cnt>=timeout_cnt_num)
//                    {
//                        ret = -1;
//                        break;
//                    }
//            }
//        data[0] = spi_i2s_data_receive(SPI0);//receive2
//		timeout_cnt = 0;
//        while ((RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE))&&(ret==0))
//            {
//                timeout_cnt++;
//                if(timeout_cnt>=timeout_cnt_num)
//                    {
//                        ret = -1;
//                        break;
//                    }
//            }
//        spi_i2s_data_transmit(SPI0, 0xFF);//transmit4
//		timeout_cnt = 0;
//        while ((RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE))&&(ret==0))
//            {
//                timeout_cnt++;
//                if(timeout_cnt>=timeout_cnt_num)
//                    {
//                        ret = -1;
//                        break;
//                    }
//            }
//        data[1] = spi_i2s_data_receive(SPI0);//receive3
//		timeout_cnt = 0;
//        while ((RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE))&&(ret==0))
//            {
//                timeout_cnt++;
//                if(timeout_cnt>=timeout_cnt_num)
//                    {
//                        ret = -1;
//                        break;
//                    }
//            }
//        data[2] = spi_i2s_data_receive(SPI0);//receive4
//    }
////  *pReadRaw = 0;
//    no_mag_warning 	= (data[1] >> 1) & 0X01;
//    over_speed 		= (data[2] >> 3) & 0X01;
//    data[1] >>= 2;
//    data[2] >>= 4;
//    *pReadRaw = (data[0] << 10) | (data[1] << 4) | data[2];
//    // NCS
//    BSP_MT6825_NCS_SET();
//    return (true);
//}



